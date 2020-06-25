# import standard libraries
import os
import sys
import numpy as np
sys.path.append('/home/lachness/.pyenv/versions/382_generic/lib/python3.8/site-packages/')

# import ros libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import AssociatedAlignedBox2D
from detection_msgs.msg import AssociatedAlignedBox2DArray

# import custom python modules
sys.path.append('/home/lachness/git_ws/quadricslam/ros/src/py_detector/py_detector')
sys.path.append('/home/lachness/git_ws/quadricslam/examples/python/example_frontend')
sys.dont_write_bytecode = True
from dataset_interfaces.scenenet_dataset import SceneNetDataset
from base.containers import Boxes, Trajectory

# import gtsam and extension
import gtsam
import quadricslam

class DatasetPublisher(Node):
    """
    Publish /detections and /poses
    """
    def __init__(self, dataset):
        super().__init__('dataset_publisher')

        # load sequence
        self.dataset = dataset
        self.sequence = dataset[0]

        # get noisy odometry
        self.true_trajectory = self.sequence.true_trajectory
        self.true_odometry = self.true_trajectory.as_odometry()
        self.noisy_odometry = self.true_odometry.add_noise(mu=0.0, sd=0.001)
        self.noisy_trajectory = self.noisy_odometry.as_trajectory(self.true_trajectory.data()[0])

        # get noisy detections
        self.true_boxes = self.sequence.true_boxes
        self.noisy_boxes = self.true_boxes.add_noise(mu=0.0, sd=2.0)

        # create publishers
        self.pose_publisher = self.create_publisher(PoseStamped, 'poses', 10)
        self.detections_publisher = self.create_publisher(AssociatedAlignedBox2DArray, 'detections', 10)

        # create timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_index = 0

    def timer_callback(self):
        """
        Every few seconds publish next pose and relevant detections
        """

        pose = self.noisy_trajectory.at(self.pose_index)
        pose_detections = self.noisy_boxes.at_pose(self.pose_index)

        header = Header()
        header.frame_id = '{}'.format(self.pose_index)
        header.stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = pose.x()
        pose_msg.pose.position.y = pose.y()
        pose_msg.pose.position.z = pose.z()
        pose_msg.pose.orientation.w = pose.rotation().quaternion()[0]
        pose_msg.pose.orientation.x = pose.rotation().quaternion()[1]
        pose_msg.pose.orientation.y = pose.rotation().quaternion()[2]
        pose_msg.pose.orientation.z = pose.rotation().quaternion()[3]
        pose_msg.header = header

        detections_msg = AssociatedAlignedBox2DArray()
        detections_msg.header = header
        detections_msg.boxes = []
        for (pose_key, object_key), box in pose_detections.items():
            box_msg = AssociatedAlignedBox2D()
            box_msg.xmin = box.xmin()
            box_msg.ymin = box.ymin()
            box_msg.xmax = box.xmax()
            box_msg.ymax = box.ymax()
            box_msg.key = int(object_key)
            detections_msg.boxes.append(box_msg)

        self.pose_publisher.publish(pose_msg)
        self.detections_publisher.publish(detections_msg)
        self.get_logger().info('Publishing pose key: {}'.format(self.pose_index))

        self.pose_index += 1

        # if we hit end of data
        if self.pose_index >= len(self.noisy_trajectory):
            self.destroy_node()
            # rclpy.shutdown()
            exit()
            
            

def main(args=None):
    dataset = SceneNetDataset(
        dataset_path = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train',
        protobuf_folder = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/data/train_protobufs',
        reader_path = '/media/lachness/DATA/Datasets/SceneNetRGBD/pySceneNetRGBD/scenenet_pb2.py',
        shapenet_path = '/media/lachness/DATA/Datasets/ShapeNet/ShapeNetCore.v2'
    )

    rclpy.init(args=args)
    dataset_publisher = DatasetPublisher(dataset)
    rclpy.spin(dataset_publisher)
    dataset_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
