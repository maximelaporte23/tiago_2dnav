#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')
        self.publisher = self.create_publisher(Twist, '/key_vel', 10)
        self.timer = self.create_timer(0.5, self.move_robot)  

    def move_robot(self):
        twist = Twist()
        twist.linear.x = 0.2  
        twist.angular.z = 0.0  
        self.publisher.publish(twist)
        self.get_logger().info(f"🚀 Commande envoyée: lin.x={twist.linear.x}, ang.z={twist.angular.z}")

def main():
    rclpy.init()
    node = MoveForward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
