# grid motion using VICON tracking data
# (control logic is in grid_motion.py)
# requires yaw calibration (vicon_yaw_calibration.py) unless bypassed

import argparse
import math
from geometry_msgs.msg import PoseStamped
from grid_motion import GridMotionNode, add_grid_arguments, run_grid
from nav_utils import (VelocityFilter, add_yaw_offset_arguments, normalize,
                       require_fresh_yaw_offset, resolve_yaw_offset, yaw_from_quaternion)


class ViconGridNode(GridMotionNode):
    label = 'vicon'
    stale_label = 'vicon pose'
    stop_message = 'stop requested'
    pose_msg_type = PoseStamped

    def __init__(self, args):
        super().__init__(args, 'grid_motion_vicon_node', args.vicon_topic, args.vicon_timeout)

    def setup_pose_source(self):
        self.yaw_offset_deg = resolve_yaw_offset(self, self.args, self.args.vicon_yaw_offset_deg)
        self.velocity = VelocityFilter(self.args.vel_filter_tau)

    def on_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = normalize(
            yaw_from_quaternion(msg.pose.orientation) + math.radians(self.yaw_offset_deg))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # velocity from filtered finite differences of vicon samples
        self.velocity.update(x, y, yaw, stamp)
        self.speed = self.velocity.v
        self.yaw_rate = self.velocity.wz
        self.store_pose(x, y, yaw, stamp)

    def waiting_message(self):
        return f'waiting for {self.args.vicon_topic}'

    def no_pose_message(self):
        return f'no vicon pose on {self.args.vicon_topic} within 10s'


def parse_args():
    p = argparse.ArgumentParser()
    add_grid_arguments(
        p, position_tol=0.005, return_position_tol=0.01, angle_tol=0.0087, homing_timeout=3.0)
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--vicon-timeout', type=float, default=0.5)
    p.add_argument('--vel-filter-tau', type=float, default=0.08)
    add_yaw_offset_arguments(p, '--vicon-yaw-offset-deg')
    return p.parse_args()


def main():
    args = parse_args()
    require_fresh_yaw_offset(args, args.vicon_yaw_offset_deg)
    run_grid(args, ViconGridNode)


if __name__ == '__main__':
    main()