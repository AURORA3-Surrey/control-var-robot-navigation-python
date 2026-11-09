# grid motion using robot wheel odometry (/odom)
# (control logic is in grid_motion.py)

import argparse
import math
import rclpy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from nav_utils import call_service, yaw_from_quaternion
from grid_motion import GridMotionNode, add_grid_arguments, run_grid


class OdomGridNode(GridMotionNode):
    label = 'odom'
    stale_label = 'odometry'
    stop_message = 'stopping robot'
    pose_msg_type = Odometry

    def __init__(self, args):
        super().__init__(args, 'grid_motion_node', args.odom_topic, args.odom_timeout)
        self.reset_client = self.create_client(Trigger, args.reset_odom_service)

    def on_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.yaw_rate = msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.store_pose(x, y, yaw, stamp)

    def waiting_message(self):
        return 'waiting for /odom'

    def no_pose_message(self):
        return 'no /odom received within 10s'

    def zero_odometry(self):
        # reset the odometry
        reset = call_service(self, self.reset_client, Trigger.Request(), 2.0, 5.0)
        if reset is None:
            self.get_logger().warn('reset odom service not found')
        elif not reset:
            self.get_logger().warn('reset odometry failed or timed out')
        else:
            deadline = self.get_time_sec() + 2.0
            settled = False
            while rclpy.ok() and self.get_time_sec() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if math.hypot(self.x, self.y) < 0.01 and abs(self.yaw) < 0.02:
                    settled = True
                    break
            if not settled:
                self.get_logger().warn(
                    f'odometry not zero after reset: ({self.x:.3f}, {self.y:.3f}, '
                    f'{math.degrees(self.yaw):+.2f} deg)')


def parse_args():
    p = argparse.ArgumentParser()
    add_grid_arguments(p, position_tol=0.01, return_position_tol=0.025, angle_tol=0.025, homing_timeout=2.0)
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--odom-timeout', type=float, default=0.5)
    p.add_argument('--reset-odom-service', default='/reset_odometry')
    return p.parse_args()


def main():
    run_grid(parse_args(), OdomGridNode)


if __name__ == '__main__':
    main()