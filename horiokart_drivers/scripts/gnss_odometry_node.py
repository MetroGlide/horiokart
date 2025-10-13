#!/usr/bin/env python3

# ===============================
# Coordinate Frames in This Node (after refactor)
# ===============================
#
# - UTM:      Global geodetic coordinate system (absolute, earth-based)
# - map:      Local map frame (static_transformでUTM→map)
# - base_link: Robot base frame (vehicle body origin, parent of gps_link)
# - gps_link: GNSS receiver sensor frame (child of base_link)
#
# - GNSS (NavSatFix): latitude/longitude/altitude (converted to UTM, i.e. UTM frame)
# - Odometry.pose: map frame (header.frame_id='map', child_frame_id='gps_link')
# - static_transform: UTM→map
# ===============================


import rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import tf_transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import pyproj

# --- TransformManager ---


class TransformManager:
    # static_transformはパラメータで与える対応点リストからのみ推定
    def __init__(self, params):
        self.static_transform = params.get('static_transform', None)
        # パラメータで与える対応点リスト（例: list of [utm_x, utm_y, odom_x, odom_y]）
        self.correspondences = params.get('correspondences', None)

    def set_static_transform(self, x, y, yaw):
        self.static_transform = (x, y, yaw)

    def estimate_transform_from_correspondences(self):
        # パラメータで与えられた対応点リストからstatic_transformを推定
        # ここでは「UTM座標→map座標」への変換（map = R*utm + t）を推定する
        if self.correspondences is None or len(self.correspondences) < 2:
            return None
        # 1. 対応点リストからGNSS（UTM）座標とmap座標を抽出
        utm_xy = np.array([[c[0], c[1]] for c in self.correspondences])
        map_xy = np.array([[c[2], c[3]] for c in self.correspondences])
        # 2. 各点群の重心を計算
        mu_utm = np.mean(utm_xy, axis=0)
        mu_map = np.mean(map_xy, axis=0)
        # 3. 重心を原点に平行移動した点群を作成
        X = utm_xy - mu_utm
        Y = map_xy - mu_map
        # 4. SVDで最適な回転行列を求める（map = R*utm + t）
        U, S, Vt = np.linalg.svd(np.dot(Y.T, X))
        R = np.dot(U, Vt)
        # 5. 反射（det(R)<0）ならVtの最終行を反転して再計算
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(U, Vt)
        # 6. 回転角（yaw）を算出
        theta = math.atan2(R[1, 0], R[0, 0])
        # 7. 並進ベクトルを算出
        t = mu_map - np.dot(R, mu_utm)
        x, y = t
        yaw = theta
        # 8. (x, y, yaw)をstatic_transformとして返す
        return (x, y, yaw)


# --- OdometryManager ---


class OdometryManager:
    def __init__(self, params):
        self.gps_frame = params.get('gps_frame_id', 'gps_link')
        self.utm_proj = pyproj.Proj(
            proj='utm', zone=54, ellps='WGS84', south=False)  # zoneは適宜変更

    def gnss_to_utm(self, gnss_msg):
        # WGS84緯度経度→UTM座標
        x, y = self.utm_proj(gnss_msg.longitude, gnss_msg.latitude)
        return x, y

# --- RosInterface ---


class GNSSOdometryNode(Node):
    # TODO: GNSS/Odometry受信異常時のエラーハンドリングを追加
    # TODO: パラメータ（UTMゾーン、サンプル数上限など）の柔軟化
    def __init__(self):
        super().__init__('gnss_odometory_node')
        # パラメータ取得
        params = {
            'static_transform': self.declare_parameter('static_transform', None).get_parameter_value().double_array_value,
            'map_frame_id': self.declare_parameter('map_frame_id', 'map').get_parameter_value().string_value,
            'gps_frame_id': self.declare_parameter('gps_frame_id', 'gps_link').get_parameter_value().string_value,

            'is_test_data': self.declare_parameter('is_test_data', False).get_parameter_value().bool_value,

            # 対応点リストはlist of [utm_x, utm_y, odom_x, odom_y]で与える
            'correspondences': [
                # [UTM座標系(x, y), map座標系(x, y)]
                [416852.97,  3993538.49, 0.130, 0.26],
                [416894.552, 3993556.40, -22.886, 37.79],
                [416853.546224, 3993538.449184, 0.130025, 0.25998],
                [416894.135621, 3993535.301766, -1.970327, 40.691986],
                [416895.132355, 3993565.376684, -31.059014, 37.89707],
            ],
        }

        if params.get('is_test_data'):
            params['correspondences'] = [
                # 平行移動+回転（45度, x=1, y=2）を含む理想的なテストデータ
                # UTM座標 (x, y) → map座標 (x', y')
                # x' = 1 + cos(π/4)*x - sin(π/4)*y
                # y' = 2 + sin(π/4)*x + cos(π/4)*y
                [0.0, 0.0, 1.0, 2.0],
                [1.0, 0.0, 1.0 + math.sqrt(2)/2, 2.0 + math.sqrt(2)/2],
                [0.0, 1.0, 1.0 - math.sqrt(2)/2, 2.0 + math.sqrt(2)/2],
                [1.0, 1.0, 1.0, 2.0 + math.sqrt(2)],
            ]

        self.transform_manager = TransformManager(params)
        self.odom_manager = OdometryManager(params)

        self.map_frame = params.get('map_frame_id')
        self.gps_frame = params.get('gps_frame_id')

        # tf2_ros Buffer/Listenerを初期化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/gnss/odom', 10)

        # サブスクライバ
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gnss_callback, 10)

        # 対応点が2点以上あればstatic_transformを自動推定
        if self.transform_manager.correspondences is not None and len(self.transform_manager.correspondences) >= 2:
            est = self.transform_manager.estimate_transform_from_correspondences()

            if est is not None:
                self.transform_manager.set_static_transform(*est)
                self.get_logger().info(
                    f"static_transform estimated from correspondences: x={est[0]:.6f}, y={est[1]:.6f}, yaw={est[2]:.6f}")

            else:
                self.transform_manager.static_transform = (0.0, 0.0, 0.0)
                self.get_logger().info('static_transform estimation failed. Initializing with (0,0,0).')
        else:
            self.transform_manager.static_transform = (0.0, 0.0, 0.0)
            self.get_logger().info('static_transform is not specified. Initializing with (0,0,0).')

        # 初期static transformをpublish
        self.publish_static_transform()

        self._tested = False

    def publish_static_transform(self):
        # static_transform (UTM→map) をtfでpublish
        tf = self.transform_manager.static_transform
        if tf is None or len(tf) != 3:
            return
        x, y, yaw = tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'utm'  # UTM座標系
        t.child_frame_id = self.map_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"Published static_transform UTM->map: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

    def gnss_callback(self, msg):
        if not self._tested:
            self._tested = True
            self.test_transform_accuracy()

        # GNSS→UTM→map (tfで変換)
        utm_x, utm_y = self.odom_manager.gnss_to_utm(msg)

        # static_transform未指定（初期パラメータ対応点なし）の場合、tfから現在のgps_link位置を取得してlog info
        if (self.transform_manager.correspondences is None or len(self.transform_manager.correspondences) < 2):
            try:
                target = self.map_frame if self.map_frame is not None else 'map'
                source = self.gps_frame if self.gps_frame is not None else 'gps_link'
                trans = self.tf_buffer.lookup_transform(target, source, Time())
                map_x_cur = trans.transform.translation.x
                map_y_cur = trans.transform.translation.y
                self.get_logger().info(
                    f"[No correspondences] GNSS UTM: ({utm_x:.3f}, {utm_y:.3f}), Current gps_link in map: ({map_x_cur:.3f}, {map_y_cur:.3f})")
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

        # utm_to_mapで変換
        map_x, map_y = self.utm_to_map(utm_x, utm_y)
        self.publish_odometry(map_x, map_y, navsat_msg=msg)
        self.get_logger().info(
            f"GNSS UTM: ({utm_x:.3f}, {utm_y:.3f}) => map: ({map_x:.3f}, {map_y:.3f})")

    def publish_odometry(self, map_x, map_y, navsat_msg=None):
        # map座標系でOdometryをpublish
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.gps_frame
        odom.pose.pose.position.x = map_x
        odom.pose.pose.position.y = map_y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        # 共分散反映
        if navsat_msg is not None and hasattr(navsat_msg, 'position_covariance'):
            cov = navsat_msg.position_covariance
            # NavSatFixの共分散は3x3(row major)→Odometryの6x6(row major)へ
            odom.pose.covariance = [
                cov[0], cov[1], cov[2], 0.0, 0.0, 0.0,
                cov[3], cov[4], cov[5], 0.0, 0.0, 0.0,
                cov[6], cov[7], cov[8], 0.0, 0.0, 0.0,
                0.0,   0.0,   0.0,   9999.0, 0.0, 0.0,
                0.0,   0.0,   0.0,   0.0,   9999.0, 0.0,
                0.0,   0.0,   0.0,   0.0,   0.0,   9999.0
            ]
        else:
            odom.pose.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 9999.0
            ]
        self.odom_pub.publish(odom)

    def utm_to_map(self, utm_x, utm_y):
        # static_transform (map = R*utm + t) を適用
        tf = self.transform_manager.static_transform
        if tf is None or len(tf) != 3:
            return utm_x, utm_y
        x, y, yaw = tf
        map_x = x + math.cos(yaw) * utm_x - math.sin(yaw) * utm_y
        map_y = y + math.sin(yaw) * utm_x + math.cos(yaw) * utm_y
        return map_x, map_y

    def test_transform_accuracy(self):
        # estimate_transform_from_correspondences実行後、correspondencesで変換の確からしさを評価
        if self.transform_manager.correspondences is None or len(self.transform_manager.correspondences) < 2:
            self.get_logger().info('No correspondences for test.')
            return

        tf = self.transform_manager.static_transform
        if tf is None or len(tf) != 3:
            self.get_logger().info('No static_transform for test.')
            return

        x, y, yaw = tf
        self.get_logger().info(
            f"[TEST] static_transform: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        for i, c in enumerate(self.transform_manager.correspondences):
            utm_x, utm_y, map_x_true, map_y_true = c
            # utm_to_mapで変換
            map_x_est, map_y_est = self.utm_to_map(utm_x, utm_y)
            err = math.hypot(map_x_est - map_x_true, map_y_est - map_y_true)
            self.get_logger().info(
                f"[TEST] pt{i}: UTM=({utm_x:.3f},{utm_y:.3f}) → map_est=({map_x_est:.3f},{map_y_est:.3f}), map_true=({map_x_true:.3f},{map_y_true:.3f}), error={err:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = GNSSOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
