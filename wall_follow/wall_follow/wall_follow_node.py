#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
# The LIDAR sends data as "best effort". We must subscribe the same way or hear nothing.
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # Settings you can change while it runs, with no rebuild.
        self.declare_parameter('kp', 0.6)                   # react to the current error
        self.declare_parameter('ki', 0.0)                   # fix slow steady drift (keep 0 at first)
        self.declare_parameter('kd', 0.1)                   # damp wobble
        self.declare_parameter('desired_distance', 0.8)     # meters to keep from the wall
        self.declare_parameter('lookahead', 0.8)            # meters ahead to predict
        self.declare_parameter('theta_deg', 50.0)           # angle between beam a and beam b
        self.declare_parameter('wall_side', 'right')        # 'right' or 'left'
        self.declare_parameter('speed', 0.3)                # requested speed in m/s
        self.declare_parameter('max_steer', 0.24)           # radians, same as the joystick steering scale
        self.declare_parameter('front_stop_distance', 0.6); self.declare_parameter('front_ignore_below', 0.25)  # stop if something is this close ahead

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.count = 0

        # Every LIDAR scan triggers scan_cb. Our commands go out on /drive.
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('wall_follow_node started')

    def p(self, name):
        return self.get_parameter(name).value

    def get_range(self, scan, angle):
        # Distance in the direction "angle" (radians, 0 = straight ahead, left is positive).
        i = int((angle - scan.angle_min) / scan.angle_increment)
        i = max(0, min(len(scan.ranges) - 1, i))
        r = scan.ranges[i]
        # A bad or missing reading counts as "far away".
        if math.isnan(r) or math.isinf(r) or r < scan.range_min:
            return scan.range_max
        return r

    def front_distance(self, scan, half_angle):
        # Closest thing in a narrow cone straight ahead, used for the stop rule.
        lo = max(0, int((-half_angle - scan.angle_min) / scan.angle_increment))
        hi = min(len(scan.ranges) - 1, int((half_angle - scan.angle_min) / scan.angle_increment))
        best = scan.range_max
        for r in scan.ranges[lo:hi + 1]:
            if max(scan.range_min, self.p('front_ignore_below')) < r < best:
                best = r
        return best

    def get_error(self, scan, theta, side, desired, lookahead):
        # Beam b points straight at the wall. Beam a points theta further forward.
        if side == 'right':
            angle_b = -math.pi / 2.0
            angle_a = angle_b + theta
        else:
            angle_b = math.pi / 2.0
            angle_a = angle_b - theta
        b = self.get_range(scan, angle_b)
        a = self.get_range(scan, angle_a)
        # alpha = angle between the car and the wall (0 when parallel).
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        d_now = b * math.cos(alpha)                    # distance to the wall right now
        d_future = d_now + lookahead * math.sin(alpha) # predicted distance a bit ahead
        return desired - d_future                      # positive = too close to the wall

    def scan_cb(self, scan):
        kp, ki, kd = self.p('kp'), self.p('ki'), self.p('kd')
        theta = math.radians(self.p('theta_deg'))
        side = self.p('wall_side')
        max_steer = self.p('max_steer')

        error = self.get_error(scan, theta, side, self.p('desired_distance'), self.p('lookahead'))

        # dt = time since the previous scan, used for the I and D terms.
        now = self.get_clock().now()
        derivative = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt > 1e-4:
                self.integral += error * dt
                self.integral = max(-1.0, min(1.0, self.integral))  # cap it so it cannot wind up
                derivative = (error - self.prev_error) / dt
        self.prev_time = now
        self.prev_error = error

        steer = kp * error + ki * self.integral + kd * derivative
        if side == 'left':
            steer = -steer      # a left wall needs the opposite steering direction
        steer = max(-max_steer, min(max_steer, steer))   # never ask for more than the servo allows

        # Stop if something is close straight ahead.
        speed = self.p('speed')
        front = self.front_distance(scan, math.radians(8.0))
        if front < self.p('front_stop_distance'):
            speed = 0.0

        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

        # Print a status line about once a second (the scan comes at about 40 Hz).
        self.count += 1
        if self.count % 40 == 0:
            self.get_logger().info(
                'err=%.2f steer=%.2f speed=%.2f front=%.2f' % (error, steer, speed, front))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
