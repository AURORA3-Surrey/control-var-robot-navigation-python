# motion commands for grid runners, commands follows a minimum-jerk position profile
# step(node, now) is called once per control tick and returns True when done
# node is the GridMotionNode which provides 'args', 'pose()', 'send()', 'tracking_correction()', 'label'
# if 'vicon' or 'odom', 'speed', 'yaw_rate', 'home' and the 'plan' queue

from nav_utils import clamp, normalize
import math

PEAK_FACTOR = 35.0/16.0    # velocity_factor at peak (tau=0.5)
HEADING_GAIN = 1.8         # yaw P-gain for heading hold
LATERAL_GAIN = 2.0         # lateral offset to yaw error
LATERAL_CLAMP = 0.35       # max lateral error [m]
HEADING_FIX_LIMIT = 0.5    # heading_fix clamp


def world_to_body(vx_world, vy_world, yaw):
    # rotate world velocity into robot body frame (omni only)
    c, s = math.cos(yaw), math.sin(yaw)
    vx_body = vx_world * c + vy_world * s
    vy_body = -vx_world * s + vy_world * c
    return vx_body, vy_body


def position_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 35 * tau**4 - 84 * tau**5 + 70 * tau**6 - 20 * tau**7


def velocity_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 140 * tau**3 - 420 * tau**4 + 420 * tau**5 - 140 * tau**6


def duration_for_peak(distance, max_speed):
    distance = abs(distance)
    if distance <= 0 or max_speed <= 0:
        return 0.0
    return PEAK_FACTOR * distance / max_speed


class Command:
    name = ''
    start_time = 0.0
    start_pose = (0.0, 0.0, 0.0)  # set by the node when the command is started

    def step(self, node, now):
        raise NotImplementedError


class Wait(Command):
    name = 'wait'

    def __init__(self, duration):
        self.duration = duration

    def step(self, node, now):
        elapsed = now - self.start_time
        node.send(0.0, 0.0, 0.0)
        return elapsed >= self.duration


class Turn(Command):
    # rotate to absolute yaw (+ or -1 controls turn direction)
    name = 'turn'

    def __init__(self, target_yaw, spin=0.0):
        self.target_yaw = target_yaw
        self.spin = spin
        self.logged = False

    def step(self, node, now):
        a = node.args
        elapsed = now - self.start_time
        x, y, yaw = node.pose()
        start_yaw = self.start_pose[2]

        raw_angle = self.target_yaw - start_yaw
        spin = self.spin
        if spin > 0 and raw_angle < 0:
            raw_angle += 2 * math.pi
        elif spin < 0 and raw_angle > 0:
            raw_angle -= 2 * math.pi
        angle = raw_angle if spin else normalize(raw_angle)

        if not self.logged:
            self.logged = True
            node.get_logger().info(
                f"turn cmd {math.degrees(angle):+.1f} deg (start yaw {math.degrees(start_yaw):+.1f}) "
                f"-> target {node.label} yaw {math.degrees(normalize(self.target_yaw)):+.1f} deg")

        T = duration_for_peak(angle, a.turn_speed)
        if T <= 0:
            return True

        homing = elapsed >= T
        tau = min(elapsed / T, 1.0)
        w_ff = 0.0 if homing else (angle / T) * velocity_factor(tau)

        turned = normalize(yaw - start_yaw)
        if abs(angle) > math.pi:
            if spin > 0 and turned < 0:
                turned += 2 * math.pi
            elif spin < 0 and turned > 0:
                turned -= 2 * math.pi

        expected = angle * position_factor(tau)
        error = expected - turned if abs(angle) > math.pi else normalize(expected - turned)

        w_correction = node.tracking_correction('turn', error, a.corr_gain, a.corr_damping_gain,
                                                 a.turn_speed, a.angle_tol)
        node.send(wz=w_ff + w_correction)

        if homing and abs(error) <= a.angle_tol and abs(node.yaw_rate) < 0.05:
            node.get_logger().info(f"landed {math.degrees(normalize(self.target_yaw - yaw)):+.2f} deg off")
            node.send(0.0, 0.0, 0.0)
            return True
        if homing and elapsed >= T + a.homing_timeout:
            node.get_logger().warn(f'turn residual {math.degrees(error):.2f} deg at timeout')
            return True
        return False


class Drive(Command):
    # drive straight line along the angle the robot had
    name = 'drive'

    def __init__(self, distance):
        self.distance = distance
        self.logged = False

    def step(self, node, now):
        a = node.args
        elapsed = now - self.start_time
        x, y, yaw = node.pose()
        start_x, start_y, start_yaw = self.start_pose

        dist = self.distance
        T = duration_for_peak(dist, a.max_speed)
        if T <= 0:
            return True

        if not self.logged:
            self.logged = True
            tgt_x = start_x + dist * math.cos(start_yaw)
            tgt_y = start_y + dist * math.sin(start_yaw)
            node.get_logger().info(
                f"drive cmd {dist:+.3f}m (start yaw {math.degrees(start_yaw):+.1f}) "
                f"{node.label} ({start_x:+.3f}, {start_y:+.3f}) -> ({tgt_x:+.3f}, {tgt_y:+.3f})")

        homing = elapsed >= T
        tau = min(elapsed / T, 1.0)
        v_ff = 0.0 if homing else (dist / T) * velocity_factor(tau)

        dx = x - start_x
        dy = y - start_y
        travelled = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)

        expected = dist * position_factor(tau)
        error = expected - travelled

        v_correction = node.tracking_correction('drive', error, a.corr_gain, a.corr_damping_gain,
                                                 a.max_speed, a.position_tol)
        heading_fix = clamp(
            HEADING_GAIN * normalize(start_yaw - yaw - clamp(LATERAL_GAIN * lateral, -LATERAL_CLAMP, LATERAL_CLAMP)),
            -HEADING_FIX_LIMIT * a.turn_speed, HEADING_FIX_LIMIT * a.turn_speed)
        node.send(vx=v_ff + v_correction, wz=heading_fix)

        if homing and abs(error) <= a.position_tol and abs(node.speed) < 0.02:
            node.get_logger().info(f"landed {error:+.3f}m along, {lateral:+.3f}m lateral off")
            node.send(0.0, 0.0, 0.0)
            return True
        if elapsed >= T + a.homing_timeout:
            node.get_logger().warn(f'drive residual {error:.3f}m at timeout')
            return True
        return False


class MoveTo(Command):
    # straight line from point a to point b (for omni robots)
    name = 'move_to'

    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.logged = False

    def step(self, node, now):
        a = node.args
        elapsed = now - self.start_time
        x, y, yaw = node.pose()
        start_x, start_y, start_yaw = self.start_pose

        if not self.logged:
            self.logged = True
            node.get_logger().info(
                f"move_to {node.label} ({start_x:+.3f}, {start_y:+.3f}) -> "
                f"({self.target_x:+.3f}, {self.target_y:+.3f})")
        dx = self.target_x - start_x
        dy = self.target_y - start_y
        T = max(duration_for_peak(dx, a.max_speed), duration_for_peak(dy, a.max_speed))
        if T <= 0:
            return True

        homing = elapsed >= T
        tau = min(elapsed / T, 1.0)
        vf = 0.0 if homing else velocity_factor(tau)
        pf = position_factor(tau)

        expected_x, expected_y = start_x + dx * pf, start_y + dy * pf
        err_x, err_y = expected_x - x, expected_y - y

        vx_corr = node.tracking_correction('move_x', err_x, a.corr_gain, a.corr_damping_gain,
                                            a.max_speed, a.position_tol)
        vy_corr = node.tracking_correction('move_y', err_y, a.corr_gain, a.corr_damping_gain,
                                            a.max_speed, a.position_tol)

        vx_world = (dx / T) * vf + vx_corr
        vy_world = (dy / T) * vf + vy_corr
        vx_body, vy_body = world_to_body(vx_world, vy_world, yaw)
        heading_fix = clamp(HEADING_GAIN * normalize(start_yaw - yaw), -HEADING_FIX_LIMIT * a.turn_speed,
                            HEADING_FIX_LIMIT * a.turn_speed)

        node.send(vx=vx_body, vy=vy_body, wz=heading_fix)

        if homing and math.hypot(err_x, err_y) <= a.position_tol and abs(node.speed) < 0.02:
            node.send(0.0, 0.0, 0.0)
            return True
        if elapsed >= T + a.homing_timeout:
            node.get_logger().warn(f'move_to residual {math.hypot(err_x, err_y):.3f}m at timeout')
            return True
        return False


class ReturnHomeDiff(Command):
    # turn to face home, drive, repeat until close enough, then face starting yaw (for diff robots)
    name = 'return_home_diff'

    def __init__(self):
        self.retries = 0
        self.aimed = False

    def step(self, node, now):
        a = node.args
        x, y, yaw = node.pose()
        hx, hy, hyaw = node.home
        dist = math.hypot(hx - x, hy - y)
        retries = self.retries
        if dist > a.return_position_tol and retries < a.return_home_max_retries:
            if not self.aimed:
                # turn to the current angle to home then reevaluate
                target_angle = math.atan2(hy - y, hx - x)
                node.get_logger().info(
                    f"return home: {node.label} ({x:+.3f}, {y:+.3f}) -> home ({hx:+.3f}, {hy:+.3f}) "
                    f"dist {dist:.3f}m angle {math.degrees(target_angle):+.1f} deg")
                self.aimed = True
                node.plan.appendleft(self)  # requeued
                node.plan.appendleft(Turn(target_angle))
            else:
                # distance measured after the turn
                node.get_logger().info(
                    f"return home attempt {retries + 1}: drive {dist:.3f}m toward home ({hx:+.3f}, {hy:+.3f})")
                self.aimed = False
                self.retries = retries + 1
                node.plan.appendleft(self)
                node.plan.appendleft(Drive(dist))
        else:
            if dist > a.return_position_tol:
                node.get_logger().warn(f'return_home_diff: giving up after {retries} attempts,'
                                       f' residual distance {dist:.3f}m')
            node.plan.appendleft(Turn(hyaw))
        return True
