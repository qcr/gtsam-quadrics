# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from detection_msgs.msg import AlignedBox2D
from detection_msgs.msg import AlignedBox2DArray

import os
import sys
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector')

from cv_bridge import CvBridge

from detector import Detector
import numpy as np

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

    def listener_callback(self, msg):
        self.get_logger().info('I heard an image with {}x{} dimensions'.format(msg.width, msg.height))

        # convert sensor_msgs/msg/Image to cv2 image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # pass image through detector 
        detections = self.detector.forward(image).astype(np.float64)

        # convert Nx85 to detection_msgs/msg/AlignedBox2DArray
        out_detections = AlignedBox2DArray()
        out_detections.header = msg.header
        out_detections.boxes = []
        for detection in detections:
            box = AlignedBox2D()
            box.xmin, box.ymin, box.xmax, box.ymax = detection[0:4]
            out_detections.boxes.append(box)
        
        # publish detections
        self.publisher.publish(out_detections)




def main(args=None):
    rclpy.init(args=args)

    ros_detector = ROSDetector()

    rclpy.spin(ros_detector)

    ros_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
