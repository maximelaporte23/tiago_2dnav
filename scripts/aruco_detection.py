#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.target_id = 32
        self.target_detected = False  
        self.distance_threshold = 100  # Distance d'arrêt en pixels
        self.mission = 0

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/key_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)
        self.tag_x = None
        self.tag_size = None

    def image_callback(self, msg):
        """ Traitement de l'image pour détecter un tag ArUco """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                if ids[i][0] == self.target_id:  
                    self.target_detected = True
                    x_center = int(np.mean(corners[i][0][:, 0]))
                    y_center = int(np.mean(corners[i][0][:, 1]))
                    self.tag_x = x_center
                    self.tag_size = np.linalg.norm(corners[i][0][0] - corners[i][0][2])  
                    cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID: {ids[i][0]} | Size: {self.tag_size:.1f}", 
                                (x_center - 30, y_center - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    def move_robot(self):
        """ Détermine le mouvement du robot """
        twist = Twist()

        if not self.target_detected:
            twist.angular.z = 0.3  
            self.get_logger().info("🔄 Recherche du tag... (rotation)")
        else:
            screen_center = 320 
            error_x = (self.tag_x - screen_center) / screen_center
            twist.angular.z = -error_x * 0.5  

            if self.mission == 0:
                if self.tag_size is not None and self.tag_size < self.distance_threshold:
                    twist.linear.x = 0.2  
                    self.get_logger().info("Avance vers le tag...")
                else:
                    twist.linear.x = 0.0  
                    twist.angular.z = 0.0
                    self.get_logger().info("Distance atteinte, arrêt.")
                    self.mission == 1

        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
