# import standard libraries
import os
import sys
import numpy as np
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')
import cv2

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # declare parameters
        self.width = self.declare_parameter('width', 320.0)
        self.height = self.declare_parameter('height', 240.0)
        self.max_fps = self.declare_parameter('fps', 10.0)

        # create camera capture
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width.value)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height.value)
        self.bridge = CvBridge()

        # create publishers
        self.image_publisher = self.create_publisher(Image, 'images', 10)

        # create timer
        timer_period = 1.0/self.max_fps.value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.n_images = 0

    def timer_callback(self):
        # get camera image 
        ret, image = self.camera.read()
        if not ret:
            return

        # convert cv2 image to msg        
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.frame_id = 'camera_frame'
        msg.header.stamp = self.get_clock().now().to_msg()

        # publish and log
        self.image_publisher.publish(msg)
        self.get_logger().info('Publishing image #{}'.format(self.n_images))
        self.n_images += 1
            

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
