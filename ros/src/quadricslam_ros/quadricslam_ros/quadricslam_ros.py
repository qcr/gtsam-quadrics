"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Ros system
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import sys
import numpy as np
import time
import argparse
import yaml

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import ObjectDetectionArray
from cv_bridge import CvBridge
import message_filters

# import custom python modules
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../quadricslam'))
sys.dont_write_bytecode = True
from base.containers import ObjectDetection
from quadricslam_online import QuadricSLAM_Online

# import gtsam and extension
import gtsam
import gtsam_quadrics


class ROSQuadricSLAM(Node):
    def __init__(self, depth, config):
        super().__init__('ROSQuadricSLAM')

        # start subscriptions
        self.pose_subscription = message_filters.Subscriber(self, PoseStamped, 'poses')
        self.detection_subscription = message_filters.Subscriber(self, ObjectDetectionArray, 'detections')
        self.image_subscription = message_filters.Subscriber(self, Image, 'image')
        self.time_synchronizer = message_filters.TimeSynchronizer([self.image_subscription, self.pose_subscription, self.detection_subscription], depth)
        self.time_synchronizer.registerCallback(self.update)    

        # load bridge between cv2 and ros2
        self.bridge = CvBridge()

        # create instance of quadricslam system
        self.SLAM = QuadricSLAM_Online(config)

    def msg2detections(self, msg):
        detections = []
        for detection in msg.detections:
                box = gtsam_quadrics.AlignedBox2(detection.box.xmin, detection.box.ymin, detection.box.xmax, detection.box.ymax) 
                detection = ObjectDetection(box, detection.objectness, detection.scores)
                detections.append(detection)
        return detections

    def msg2pose(self, msg):
        point = gtsam.Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rot = gtsam.Rot3.Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        return gtsam.Pose3(rot, point)

    def msg2image(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    def msg2time(self, msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)*1e-9

    def update(self, image_msg, pose_msg, detections_msg):
        update_start = time.time()
        self.get_logger().info('Update started')

        # convert msgs to data
        image = self.msg2image(image_msg)
        camera_pose = self.msg2pose(pose_msg)
        float_time = self.msg2time(detections_msg)
        image_detections = self.msg2detections(detections_msg)

        # incrementally update slam system 
        self.SLAM.update(image, image_detections, camera_pose)








def main(main_args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', dest='config', type=str, required=True,
                        help='path to the camera configuration file')
    parser.add_argument('--depth', dest='depth', type=int, default=10, 
                        help='the queue depth to store topic messages')
    args = parser.parse_args()

    # load configuration
    config = yaml.safe_load(open(args.config, 'r'))
    
    # init ros
    rclpy.init(args=main_args)

    # create node
    system = ROSQuadricSLAM(args.depth, config)

    # spin node
    rclpy.spin(system)

    # shutdown 
    system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
