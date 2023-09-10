#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import ros2_numpy as rnp
import open3d as o3d


class PointCloudDownsampler(Node):

    def __init__(self):
        super().__init__("pc_downsampling_node")

        # サブスクライバーの作成
        self.subscription = self.create_subscription(
            PointCloud2, "/cloud", self.callback, 10
        )

        # パブリッシャーの作成
        self.publisher = self.create_publisher(
            PointCloud2, "/downsampled_cloud", 10
        )

    def callback(self, msg: PointCloud2):
        # # 点群の読み込み
        # pcd = o3d.io.read_point_cloud(msg.data)

        # # ダウンサンプリング
        # downsampled_pcd = o3d.geometry.voxel_downsample(pcd, voxel_size=0.05)

        # 点群データの座標をOpen3Dの形式に変換
        xyz = rnp.numpify(msg)

        # ダウンサンプリングした点群データをOpen3DのPointCloudオブジェクトに変換
        downsampled_pcd = o3d.geometry.PointCloud()
        downsampled_pcd.points = o3d.utility.Vector3dVector(xyz)

        downsampled_pcd = o3d.geometry.voxel_downsample(
            voxel_size=0.05)

        # ダウンサンプリングした点群をPointCloud2型に変換
        downsampled_cloud = PointCloud2()
        downsampled_cloud.header = msg.header
        downsampled_cloud.height = 1
        downsampled_cloud.width = len(downsampled_pcd.points)
        downsampled_cloud.fields = msg.fields
        downsampled_cloud.is_bigendian = msg.is_bigendian
        downsampled_cloud.point_step = msg.point_step
        downsampled_cloud.row_step = msg.row_step
        downsampled_cloud.data = downsampled_pcd.points

        # パブリッシュ
        self.publisher.publish(downsampled_cloud)


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudDownsampler()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
