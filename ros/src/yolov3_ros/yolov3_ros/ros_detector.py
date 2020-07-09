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
from detection_msgs.msg import ObjectDetection
from detection_msgs.msg import ObjectDetectionArray
from cv_bridge import CvBridge

# import custom modules 
sys.dont_write_bytecode = True
from yolov3_ros.detector import Detector

# import quadricslam modules
import gtsam_quadrics
sys.path.append(os.path.dirname(os.path.realpath(__file__)).split('quadricslam/ros')[0]+'quadricslam/quadricslam')
from visualization.drawing import CV2Drawing

class ROSDetector(Node):

    def __init__(self):
        super().__init__('ROSDetector')

        # set parameters
        self.visualize = self.declare_parameter('visualize', True).value
        weights_path = self.declare_parameter('weights', '/home/lachness/git_ws/PyTorch-YOLOv3/weights/yolov3.weights').value
        config_path = self.declare_parameter('config', '/home/lachness/git_ws/PyTorch-YOLOv3/config/yolov3.cfg').value
        classes_path = self.declare_parameter('classes', '/home/lachness/git_ws/PyTorch-YOLOv3/data/coco.names').value

        # create image subscriber
        self.subscription = self.create_subscription(Image, 'image', self.listener_callback, 10)

        # create detections publisher 
        self.publisher = self.create_publisher(ObjectDetectionArray, 'detections', 10)

        # create object detector 
        self.detector = Detector(weights_path, config_path, classes_path)

        # store cvbridge
        self.bridge = CvBridge()

        # log info 
        self.get_logger().info('Starting Detector with device "{}"'.format(self.detector.device))


    def listener_callback(self, msg):
        # convert sensor_msgs/msg/Image to cv2 image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # pass image through detector 
        detections = self.detector.forward(image)

        # convert to float64 (from float32)
        detections = detections.astype(np.float64)

        # convert Nx85 to detection_msgs/msg/AlignedBox2DArray
        out_detections = ObjectDetectionArray()
        out_detections.header = msg.header
        out_detections.detections = []
        for detection in detections:
            d = ObjectDetection()
            d.box.xmin, d.box.ymin, d.box.xmax, d.box.ymax = detection[0:4]
            d.objectness = detection[4]
            d.scores = detection[5:]
            out_detections.detections.append(d)
        
        # publish detections
        float_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1e-9
        self.get_logger().info('Publishing {} detections from time {}'.format(len(detections), float_time))
        self.publisher.publish(out_detections)

        # draw detections
        if self.visualize:
            self.draw_detections(image, detections)


    def draw_detections(self, image, detections):
        drawing = CV2Drawing(image)
        for detection in detections:
            box = gtsam_quadrics.AlignedBox2(*detection[0:4])
            class_scores = detection[5:] * detection[4]
            text = '{}:{:.2f}'.format(self.detector.classes[np.argmax(class_scores)], np.max(class_scores))
            drawing.box_and_text(box, (255,255,0), text, (0,0,0))
        cv2.imshow('detections', image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    ros_detector = ROSDetector()
    rclpy.spin(ros_detector)
    ros_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
