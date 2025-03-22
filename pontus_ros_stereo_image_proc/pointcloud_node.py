#!/usr/bin/env python 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class Pointcloud(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        self.left_camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/left/camera_info',
            self.left_camera_info_callback,
            10
        )

        self.right_camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/right/camera_info',
            self.right_camera_info_callback,
            10
        )

        self.disparity_image_subscription = self.create_subscription(
            Image,
            '/disparity_image',
            self.disparity_callback,
            10
        )

        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/points2',
            10
        )

        self.cx = None
        self.cy = None
        self.f = None
        self.Tx = None
        self.bridge = CvBridge()
    
    def left_camera_info_callback(self, msg: CameraInfo):
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.f = msg.k[0]
    
    def right_camera_info_callback(self, msg: CameraInfo):
        self.Tx = -msg.p[3]
    
    def disparity_callback(self, msg: Image):
        if self.cx is None or self.Tx is None:
            self.get_logger().warn("Waiting on camera intrinsics, skipping")
            return
        
        disparity_image = self.bridge.imgmsg_to_cv2(msg)
        rows, cols = disparity_image.shape
        row_indices, col_indices = np.meshgrid(np.arange(rows), np.arange(cols), indexing='ij')
        disparities = disparity_image.flatten()
        rows_flat = row_indices.flatten()
        cols_flat = col_indices.flatten()
        valid_mask = disparities > 0
        valid_disparities = disparities[valid_mask]
        valid_rows = rows_flat[valid_mask]
        valid_cols = cols_flat[valid_mask]
        z = self.Tx / valid_disparities
        x = (valid_cols - self.cx) * z / self.f
        y = (valid_rows - self.cy) * z / self.f

        points = np.vstack((x, y, z)).T
        # Rotation matrix from optical to body frame
        R = np.array([[0, 0, 1],
                      [-1, 0, 0],
                      [0, -1, 0]])
        points_body = np.dot(points, R.T)
        # Create a PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id
        point_cloud_msg = pc2.create_cloud_xyz32(header, points_body)
        
        # Publish the message
        self.point_cloud_pub.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Pointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()