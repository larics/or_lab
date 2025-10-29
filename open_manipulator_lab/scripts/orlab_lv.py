#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from math import pi, floor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lv_priprema import LV2526_priprema


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
NUM_JOINTS = len(JOINT_NAMES)

class ORLAB_OpenManipulator(Node):
    def __init__(self):
        super().__init__('ROBLAB_open_manipulator')

        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_min = [-3, -1.5, -1.4, -1.5]
        self.joint_max = [3, 1.5, 1, 1.5]

        print("Initiated")

    def wrap2PI(self, x):
        return (x-2*pi*floor(x/(2*pi)+0.5))

    def moveRobot(self, q, t):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = np.clip(q, self.joint_min, self.joint_max)
        
        # Set the time_from_start (how long the robot has to reach this goal)
        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        msg.points.append(point)
        
        # Log the command being sent
        self.get_logger().info(f'Prepared trajectory for: {JOINT_NAMES}')
        self.get_logger().info(f'Target positions: {point.positions}')
        self.get_logger().info(f'Execution duration: {t} seconds')
        
        input("Press Enter to execute")
        
        # Publish the complete message
        self.publisher_.publish(msg)
        self.get_logger().info('Published trajectory command successfully. Shutting down...')

def main(args=None):
    rclpy.init(args=args)
    node = ORLAB_OpenManipulator()
    kinematics_node = LV2526_priprema()

    q = [0, 0, 0, 0]
    
    sol_dk = kinematics_node.get_dk(q)
    print('DK: ', sol_dk)
    
    sol_ik = kinematics_node.get_ik(sol_dk)
    print('IK: ', sol_ik)

    sol_ik_best = kinematics_node.get_closest_ik(sol_ik, q)
    print('Best: ', sol_ik_best)

    node.moveRobot(q, 1.0)


if __name__ == '__main__':
    main()
