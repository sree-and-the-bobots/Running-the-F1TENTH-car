#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        # --- seeing and avoiding ---
        self.declare_parameter('max_range', 4.0)        # m, how far ahead it looks
        self.declare_parameter('ignore_below', 0.35)    # m, closer readings = the car's own body
        self.declare_parameter('fov_deg', 90.0)         # degrees left and right of straight ahead
        self.declare_parameter('half_width', 0.22)      # m, half the car's width plus a margin
        self.declare_parameter('disparity', 0.4)        # m, a jump bigger than this is an obstacle edge
        self.declare_parameter('smooth_window', 1)      # whole number, 1 = no smoothing
        self.declare_parameter('tie_margin', 0.3)       # m, points this close to the deepest count as ties
        # --- steering ---
        self.declare_parameter('steer_gain', 0.8)
        self.declare_parameter('steer_sign', -1.0)       # -1.0 if the wheels turn the wrong way
        self.declare_parameter('steer_alpha', 0.6)      # 1.0 = no smoothing
        self.declare_parameter('max_steer', 0.24)       # rad
        # --- speed ---
        self.declare_parameter('speed_min', 0.5)
        self.declare_parameter('speed_max', 0.9)
        self.declare_parameter('slow_zone', 1.5)        # m, start slowing below this distance ahead
        # --- stop and reverse ---
        self.declare_parameter('stop_distance', 0.6)    # m, "blocked" if something is this close ahead
        self.declare_parameter('stop_cone_deg', 12.0)
        self.declare_parameter('box_in_distance', 0.8); self.declare_parameter('hard_stop_distance', 0.45)  # m, "blocked" if even the deepest point is this close
        self.declare_parameter('reverse_enable', True)
        self.declare_parameter('reverse_speed', 0.5)
        self.declare_parameter('reverse_time', 1.2)     # s
        self.declare_parameter('brake_time', 0.3)       # s
        self.declare_parameter('cooldown_time', 0.8)    # s
        self.declare_parameter('max_reverses', 3)       # in a row, then it stops

        self.prev_steer = 0.0
        self.count = 0
        self.state = 'drive'
        self.t0 = 0.0
        self.rev_side = 1.0
        self.cooldown_until = 0.0
        self.last_trigger = -100.0
        self.streak = 0

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('reactive_node (follow the gap + reverse) started')

    def p(self, name):
        return self.get_parameter(name).value

    def publish(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def scan_cb(self, scan):
        self.count += 1
        log = (self.count % 40 == 0)

        inc = scan.angle_increment
        n = len(scan.ranges)
        fov = math.radians(self.p('fov_deg'))
        lo = max(0, int((-fov - scan.angle_min) / inc))
        hi = min(n - 1, int((fov - scan.angle_min) / inc))
        if hi - lo < 20:
            return

        def ang(k):
            return scan.angle_min + (lo + k) * inc   # + is left, - is right

        # STEP 1: clean. Bad readings, the car's body and far points become max_r (free).
        max_r = self.p('max_range')
        ignore = max(self.p('ignore_below'), scan.range_min)
        r = []
        for i in range(lo, hi + 1):
            v = scan.ranges[i]
            if math.isnan(v) or math.isinf(v) or v < ignore:
                v = max_r
            r.append(min(v, max_r))
        m = len(r)

        w = int(self.p('smooth_window'))
        if w > 1:
            half = w // 2
            sm = []
            for i in range(m):
                j0 = max(0, i - half)
                j1 = min(m, i + half + 1)
                sm.append(sum(r[j0:j1]) / (j1 - j0))
            r = sm

        # STEP 2: disparity extender. At every sharp jump, push the NEARER edge
        # outward by the car's half-width so the car clears the corner.
        ext = list(r)
        hw = self.p('half_width')
        dth = self.p('disparity')
        for i in range(m - 1):
            d = r[i + 1] - r[i]
            if abs(d) > dth:
                if d > 0:
                    near = r[i]
                    cnt = int(math.ceil(math.atan2(hw, max(near, 0.1)) / inc))
                    for k in range(i + 1, min(m, i + 1 + cnt)):
                        if ext[k] > near:
                            ext[k] = near
                else:
                    near = r[i + 1]
                    cnt = int(math.ceil(math.atan2(hw, max(near, 0.1)) / inc))
                    for k in range(max(0, i - cnt + 1), i + 1):
                        if ext[k] > near:
                            ext[k] = near

        # Distance straight ahead (narrow cone), and the deepest open point.
        c0 = int((0.0 - scan.angle_min) / inc) - lo
        cone = int(math.radians(self.p('stop_cone_deg')) / inc)
        f0 = max(0, c0 - cone)
        f1 = min(m - 1, c0 + cone)
        front = min(r[f0:f1 + 1]) if f1 >= f0 else max_r
        mx = max(ext)

        # STEP 3: target = deepest point; ties go to the one closest to straight ahead.
        tie = self.p('tie_margin')
        best_k = 0
        best_abs = 1e9
        for k in range(m):
            if ext[k] >= mx - tie:
                a = abs(ang(k))
                if a < best_abs:
                    best_abs = a
                    best_k = k
        angle = ang(best_k)

        # Which side is more open (used to decide the reverse direction).
        left = sum(ext[c0:]) / max(1, m - c0)
        right = sum(ext[:c0]) / max(1, c0)
        side = 1.0 if left >= right else -1.0

        stop_d = self.p('stop_distance')
        blocked = (front < self.p('hard_stop_distance')) or ((front < stop_d) and (mx < self.p('box_in_distance')))
        rev_on = bool(self.p('reverse_enable'))
        now = self.get_clock().now().nanoseconds / 1e9
        max_steer = self.p('max_steer')
        sgn = self.p('steer_sign')

        # ---- state machine: drive -> brake -> reverse -> drive ----
        if self.state == 'brake':
            self.publish(0.0, 0.0)
            if now - self.t0 >= self.p('brake_time'):
                self.state = 'reverse'
                self.t0 = now
            return

        if self.state == 'reverse':
            # Reversing flips the turn: to swing the nose toward the open side,
            # point the wheels AWAY from it.
            self.publish(-sgn * self.rev_side * max_steer, -self.p('reverse_speed'))
            if now - self.t0 >= self.p('reverse_time'):
                self.state = 'drive'
                self.cooldown_until = now + self.p('cooldown_time')
                self.prev_steer = 0.0
            return

        if self.state == 'stuck':
            self.publish(0.0, 0.0)
            if not blocked:
                self.state = 'drive'
                self.streak = 0
            return

        # state == 'drive'
        if blocked:
            if (not rev_on) or now < self.cooldown_until:
                self.publish(self.prev_steer, 0.0)
                if log:
                    self.get_logger().info('blocked, holding. front=%.2f deepest=%.2f' % (front, mx))
                return
            if now - self.last_trigger > 6.0:
                self.streak = 0
            self.last_trigger = now
            self.streak += 1
            if self.streak > int(self.p('max_reverses')):
                self.state = 'stuck'
                self.get_logger().info('WARNING: boxed in after several reverses, stopped. Move the car or the obstacle.')
                self.publish(0.0, 0.0)
                return
            self.rev_side = side
            self.state = 'brake'
            self.t0 = now
            self.publish(0.0, 0.0)
            self.get_logger().info('blocked (front=%.2f deepest=%.2f): braking, then reversing %s'
                                   % (front, mx, 'toward the left' if side > 0 else 'toward the right'))
            return

        # Normal driving: steer toward the target.
        steer = sgn * self.p('steer_gain') * angle
        steer = max(-max_steer, min(max_steer, steer))
        a = self.p('steer_alpha')
        steer = a * steer + (1.0 - a) * self.prev_steer
        self.prev_steer = steer

        sp_min = self.p('speed_min')
        sp_max = self.p('speed_max')
        room = (front - stop_d) / max(self.p('slow_zone') - stop_d, 0.1)
        room = max(0.0, min(1.0, room))
        straight = 1.0 - min(1.0, abs(steer) / max_steer) if max_steer > 1e-6 else 1.0
        speed = sp_min + (sp_max - sp_min) * room * straight

        self.publish(steer, speed)
        if log:
            self.get_logger().info('drive angle=%.2f steer=%.2f speed=%.2f front=%.2f deepest=%.2f'
                                   % (angle, steer, speed, front, mx))


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
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
