#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        time.sleep(1)
        self.extend_arm()
        time.sleep(3)
        self.retract_arm()

    def extend_arm(self):
        """ Commande pour tendre le bras vers l'avant """
        self.get_logger().info("Étend le bras...")
        msg = JointTrajectory()
        msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.2, 1.5, 0.0, 0.0]  
        point.time_from_start.sec = 2  
        msg.points.append(point)
        self.arm_publisher.publish(msg)

    def retract_arm(self):
        """ Commande pour ramener le bras vers soi """
        self.get_logger().info("Replie le bras...")
        msg = JointTrajectory()
        msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5, 0.5, 0.0, 0.0]  
        point.time_from_start.sec = 2  
        msg.points.append(point)
        self.arm_publisher.publish(msg)

def main():
    rclpy.init()
    node = ArmMover()
    time.sleep(5)  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
