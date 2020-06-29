"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Trajectory, Quadrics, Boxes, Odometry containers
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
import numpy as np
import cv2

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from detection_msgs.msg import AlignedBox2D
from detection_msgs.msg import AlignedBox2DArray
from cv_bridge import CvBridge

# import custom modules 
sys.dont_write_bytecode = True
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector')
from detector import Detector

class ROSDetector(Node):

    def __init__(self):
        super().__init__('ROSDetector')

        # create image subscriber
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # create detections publisher 
        self.publisher = self.create_publisher(AlignedBox2DArray, 'detections', 10)

        # create object detector and cvbridge
        self.detector = Detector()
        self.bridge = CvBridge()

        # log info 
        self.get_logger().info('Starting Detector with device "{}"'.format(self.detector.device))


    def listener_callback(self, msg):
        # convert sensor_msgs/msg/Image to cv2 image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # pass image through detector 
        detections = self.detector.forward(image)

        if detections is None:
            detections = np.array([])

        # convert to float64 (from float32)
        detections = detections.astype(np.float64)

        # convert Nx85 to detection_msgs/msg/AlignedBox2DArray
        out_detections = AlignedBox2DArray()
        out_detections.header = msg.header
        out_detections.boxes = []
        for detection in detections:
            box = AlignedBox2D()
            box.xmin, box.ymin, box.xmax, box.ymax = detection[0:4]
            out_detections.boxes.append(box)
        
        # publish detections
        float_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1e-9
        self.get_logger().info('Publishing {} detections from time {}'.format(len(detections), float_time))
        self.publisher.publish(out_detections)


def main(args=None):
    rclpy.init(args=args)
    ros_detector = ROSDetector()
    rclpy.spin(ros_detector)
    ros_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
