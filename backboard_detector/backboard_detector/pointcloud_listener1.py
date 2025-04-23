#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class BackboardDetector(Node):
    def __init__(self):
        super().__init__('backboard_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Point, '/centers_listener', 10)
        self.get_logger().info('BackboardDetector node with centerpublishing started.')
        self.keepalive_timer = self.create_timer(1.0, lambda: None)
    
    def listener_callback(self, msg):
        points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        if not points:
            return
        points_np = np.array([[p.x, p.y, p.z] for p in points])
        
        # Open3Dへ変換
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)
        
        try:
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=0.02,  # 点が推定された平面からどれだけ離れていてもよいか
                ransac_n=3,               # ランダムに選ばれる点の数
                num_iterations=1000       # 反復回数
            )
        except RuntimeError:
            self.get_logger().warn('Plane segmentation failed.')
            return
        # ax+by+cz+d=0 の形で平面の方程式を取得
        [a, b, c, d] = plane_model
        normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        inlier_cloud = pcd.select_by_index(inliers)
        bounds = inlier_cloud.get_axis_aligned_bounding_box()  # 点群から軸に平行なBoundingBoxを取得
        extent = bounds.get_extent()  # [幅, 高さ, 奥行き]
        center = bounds.get_center()
        
        is_backboard = (
            1.3 < extent[0] < 1.7 and  # 横幅がほぼ1500mm
            0.8 < extent[1] < 1.1 and  # 高さが約900mm
            2.3 < center[2] < 2.6 and  # 高さが約2.43m
            abs(normal[2]) < 0.3 and   # Z軸方向への傾きが少ない（垂直）
            abs(normal[1]) > 0.7       # Y方向に立っている
        )
        
        if is_backboard:
            center_msg = Point()
            center_msg.x = float(center[0])
            center_msg.y = float(center[1])
            center_msg.z = float(center[2])
            self.publisher_.publish(center_msg)
            self.get_logger().info(f"Backboard center: ({center_msg.x:.2f}, {center_msg.y:.2f}, {center_msg.z:.2f})")


def main(args=None):
    print('Hi from backboard_detector.')
    rclpy.init(args=args)
    backboard_detector = BackboardDetector()
    rclpy.spin(backboard_detector)
    backboard_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
