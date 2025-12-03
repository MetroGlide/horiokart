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
import yaml
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from dataclasses import dataclass
from typing import List, Optional, Any, Type
import numpy as np
import math
import pyproj
from ublox_msgs.msg import NavPVT

# --- TransformManager ---


class TransformManager:
    # 複数の static_transform を管理し、ラベルで切替可能にする
    def __init__(self, params):
        # 単一の static_transform (x,y,yaw) — 現在アクティブなものを保持
        self.static_transform = None
        # 全 transform を辞書 label -> (x,y,yaw) で保持
        self.transforms = {}
        # 現在のラベル名
        self.active_label = None
        # パラメータで与える対応点リスト（例: list of [utm_x, utm_y, odom_x, odom_y]）
        self.correspondences = params.get('correspondences', None)

    def set_static_transform(self, x, y, yaw):
        self.static_transform = (x, y, yaw)

    def load_transforms_from_file(self, path):
        # YAML ファイルを読み、transforms をロードする
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError as e:
            print(f"[TransformManager] File not found: {e}")
            return False
        except PermissionError as e:
            print(f"[TransformManager] Permission error: {e}")
            return False
        except yaml.YAMLError as e:
            print(f"[TransformManager] YAML error: {e}")
            return False

        if not data:
            return False

        transforms = {}
        entries = data.get('transforms') or []
        for e in entries:
            label = e.get('label')
            tf = e.get('transform')
            if label and tf and len(tf) == 3:
                transforms[label] = (float(tf[0]), float(tf[1]), float(tf[2]))

        if transforms:
            self.transforms = transforms
            return True
        return False

    def set_active_label(self, label):
        # アクティブラベルを切替え、static_transform を更新する
        if label not in self.transforms:
            return False
        self.active_label = label
        self.static_transform = self.transforms[label]
        return True

    def get_labels(self):
        return list(self.transforms.keys())

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
        self.gps_frame = params.get('gps_frame_id')
        utm_zone = params.get('utm_zone')
        self.utm_proj = pyproj.Proj(
            proj='utm', zone=utm_zone, ellps='WGS84', south=False)

    def gnss_to_utm(self, longitude, latitude):
        # WGS84 緯度経度 -> UTM 座標
        # 引数: longitude (deg), latitude (deg)
        x, y = self.utm_proj(longitude, latitude)
        return x, y


# --- GNSS Handler Abstractions (single-file implementation) ---


@dataclass
class HandlerResult:
    lat_deg: float
    lon_deg: float
    alt_m: Optional[float]
    yaw_rad: Optional[float]
    quaternion: Optional[Quaternion]
    covariance: List[float]  # length 36, row-major 6x6
    source_msg: Optional[Any] = None


class BaseGNSSHandler:
    """Base class for GNSS handlers.

    Subclasses must set msg_type to the ROS message class they handle and
    implement process(msg) -> HandlerResult.
    """

    msg_type: Type = None
    # Default topic for this handler; subclasses may override or set via params
    topic: Optional[str] = None

    def __init__(self, params: dict):
        self.params = params

    def process(self, msg) -> HandlerResult:
        raise NotImplementedError()


class NavSatFixHandler(BaseGNSSHandler):
    msg_type = NavSatFix
    topic = '/gps/fix'

    def process(self, msg) -> HandlerResult:
        # Expect msg.latitude, msg.longitude, msg.altitude and msg.position_covariance
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude if self.params.get('use_altitude') else None

        # Use validated global default covariance (length 36). Handlers are
        # responsible for returning a full 36-element covariance matrix.
        cov = list(self.params['default_covariance'])
        # If message provides a 3x3 position covariance, copy into 6x6 position block
        if msg.position_covariance is not None:
            ncov = list(msg.position_covariance)
            cov[0] = ncov[0]
            cov[1] = ncov[1]
            cov[2] = ncov[2]
            cov[6] = ncov[3]
            cov[7] = ncov[4]
            cov[8] = ncov[5]
            cov[12] = ncov[6]
            cov[13] = ncov[7]
            cov[14] = ncov[8]

        # NavSatFix provides no orientation by itself
        return HandlerResult(
            lat_deg=lat,
            lon_deg=lon,
            alt_m=alt,
            yaw_rad=None,
            quaternion=None,
            covariance=cov,
            source_msg=msg,
        )


class NavPVTHandler(BaseGNSSHandler):
    msg_type = NavPVT
    topic = '/ublox/navpvt'

    def __init__(self, params: dict):
        super().__init__(params)
        # heading smoothing state per-handler
        self.heading_sin = 0.0
        self.heading_cos = 0.0

    def process(self, msg) -> HandlerResult:
        # Convert lon/lat
        lon = msg.lon * 1e-7
        lat = msg.lat * 1e-7

        # Altitude: use if enabled by parameter; NavPVT.height is in mm -> meters
        alt = float(msg.height) / \
            1000.0 if self.params.get('use_altitude') else None

        # Determine heading source: prefer head_veh if FLAGS_HEAD_VEH_VALID set
        FLAGS_HEAD_VEH_VALID = 32
        if (msg.flags & FLAGS_HEAD_VEH_VALID) != 0:
            raw_deg = msg.head_veh * 1e-5
            src = 'head_veh'
        else:
            raw_deg = msg.heading * 1e-5
            src = 'heading'

        yaw_rad = None

        # speed guard for motion heading (g_speed: mm/s -> m/s)
        speed = float(msg.g_speed) / 1000.0

        if raw_deg is not None:
            if src == 'heading' and speed is not None and speed < self.params.get('min_speed_for_heading'):
                # skip motion heading when nearly stationary
                yaw_rad = None
            else:
                yaw_rad = math.radians(raw_deg)
                if self.params.get('apply_heading_invert'):
                    yaw_rad = -yaw_rad
                if self.params.get('apply_heading_add_pi'):
                    yaw_rad = yaw_rad + math.pi
                if self.params.get('apply_heading_offset'):
                    yaw_rad = yaw_rad + \
                        math.radians(self.params.get('heading_offset_deg'))

                # Smooth heading using circular EMA
                s = math.sin(yaw_rad)
                c = math.cos(yaw_rad)
                if self.heading_sin == 0.0 and self.heading_cos == 0.0:
                    self.heading_sin = s
                    self.heading_cos = c
                else:
                    a = self.params.get('heading_smoothing_alpha')
                    self.heading_sin = a * s + (1 - a) * self.heading_sin
                    self.heading_cos = a * c + (1 - a) * self.heading_cos
                yaw_rad = math.atan2(self.heading_sin, self.heading_cos)

        # Use validated global default covariance and override entries using NavPVT fields
        cov = list(self.params['default_covariance'])
        _scale = 1.5
        _bias = 1.5  # [m]
        if msg.h_acc is not None and msg.h_acc > 0:
            pos_std = (msg.h_acc / 1000.0) * \
                self.params['navpvt_hacc_to_pos_std_scale']
            pos_var = pos_std * pos_std
            cov[0] = pos_var * _scale + _bias ** 2
            cov[7] = pos_var * _scale + _bias ** 2
        if msg.v_acc is not None and msg.v_acc > 0:
            z_std = (msg.v_acc / 1000.0) * \
                self.params['navpvt_vacc_to_pos_std_scale']
            z_var = z_std * z_std
            cov[14] = z_var
        if msg.head_acc is not None and msg.head_acc > 0:
            headacc_deg = msg.head_acc * 1e-5
            yaw_std = math.radians(headacc_deg) * \
                self.params['navpvt_headacc_to_yaw_std_scale']
            cov[35] = yaw_std * yaw_std

        # Build quaternion if yaw available
        q = None
        if yaw_rad is not None:
            q_arr = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
            q = Quaternion()
            q.x = q_arr[0]
            q.y = q_arr[1]
            q.z = q_arr[2]
            q.w = q_arr[3]

        return HandlerResult(
            lat_deg=lat,
            lon_deg=lon,
            alt_m=alt,
            yaw_rad=yaw_rad,
            quaternion=q,
            covariance=cov,
            source_msg=msg,
        )


# --- RosInterface ---


class GNSSOdometryNode(Node):
    # TODO: GNSS/Odometry受信異常時のエラーハンドリングを追加
    # TODO: パラメータ（UTMゾーン、サンプル数上限など）の柔軟化
    def __init__(self):
        super().__init__('gnss_odometry_node')
        # パラメータ取得
        # 注意: 数値パラメータには単位をコメントで明記しています。
        params = {
            'static_transform': self.declare_parameter('static_transform', None).get_parameter_value().double_array_value,
            'map_frame_id': self.declare_parameter('map_frame_id', 'map').get_parameter_value().string_value,
            'gps_frame_id': self.declare_parameter('gps_frame_id', 'gps_link').get_parameter_value().string_value,

            'is_test_data': self.declare_parameter('is_test_data', False).get_parameter_value().bool_value,
            'utm_zone': self.declare_parameter('utm_zone', 54).get_parameter_value().integer_value,

            # input selection: 'navsatfix' or 'navpvt'
            'gnss_input': self.declare_parameter('gnss_input', 'navsatfix').get_parameter_value().string_value,
            # min_speed_for_heading: m/s (地上速度がこの閾値未満の場合、motion heading は信用しない)
            'min_speed_for_heading': self.declare_parameter('min_speed_for_heading', 0.5).get_parameter_value().double_value,
            # heading_smoothing_alpha: unitless (0..1), 環状EMAのα
            'heading_smoothing_alpha': self.declare_parameter('heading_smoothing_alpha', 0.6).get_parameter_value().double_value,
            # Heading correction options
            # apply_heading_offset: bool, enable applying heading_offset_deg (degrees)
            'apply_heading_offset': self.declare_parameter('apply_heading_offset', False).get_parameter_value().bool_value,
            # heading_offset_deg: degrees, applied if apply_heading_offset is True
            'heading_offset_deg': self.declare_parameter('heading_offset_deg', 0.0).get_parameter_value().double_value,
            # apply_heading_invert: bool, multiply heading by -1 when True
            'apply_heading_invert': self.declare_parameter('apply_heading_invert', True).get_parameter_value().bool_value,
            # apply_heading_add_pi: bool, add 180 deg (pi rad) to heading when True
            'apply_heading_add_pi': self.declare_parameter('apply_heading_add_pi', False).get_parameter_value().bool_value,

            # --- Covariance source parameters ---
            # NavPVT-based covariance parameters
            # navpvt_hacc_to_pos_std_scale: unitless scale applied to hAcc (mm -> m) to derive pos std [m]
            'navpvt_hacc_to_pos_std_scale': self.declare_parameter('navpvt_hacc_to_pos_std_scale', 1.0).get_parameter_value().double_value,
            # navpvt_vacc_to_pos_std_scale: unitless scale applied to vAcc (mm -> m) to derive z std [m]
            'navpvt_vacc_to_pos_std_scale': self.declare_parameter('navpvt_vacc_to_pos_std_scale', 1.0).get_parameter_value().double_value,
            # navpvt_headacc_to_yaw_std_scale: unitless scale applied to headAcc->rad to derive yaw standard deviation
            'navpvt_headacc_to_yaw_std_scale': self.declare_parameter('navpvt_headacc_to_yaw_std_scale', 1.0).get_parameter_value().double_value,
            # Global default covariance (36 elements: row-major 6x6). If provided and length==36,
            # handlers and node will prefer this array as the default covariance.
            'default_covariance': self.declare_parameter('default_covariance', [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 9999.0
            ]).get_parameter_value().double_array_value,

            # Use altitude/height value from messages. When False, handlers will
            # ignore altitude and set alt_m=None. Default: False (disabled).
            'use_altitude': self.declare_parameter('use_altitude', False).get_parameter_value().bool_value,

            # 対応点リストは list of [utm_x, utm_y, odom_x, odom_y] で与える (単位: m)
            'correspondences': [
                # [UTM座標系(x, y), map座標系(x, y)]
                [416860.455628, 3993538.760756, 19.648010, 21.072767],
                [416896.252099, 3993539.142067, 18.218765, 56.421684],
            ],
        }

        # 複数 transform を定義した YAML ファイル (相対パス可)
        params['static_transforms_file'] = self.declare_parameter(
            'static_transforms_file', '/root/ros2_data/map/gnss_to_map_static_transforms.yaml').get_parameter_value().string_value
        # 起動時に選択するラベル (空ならファイル内の最初を使用)
        params['static_transform_label'] = self.declare_parameter(
            'static_transform_label', '').get_parameter_value().string_value

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

        # Store params centrally so code uses self.params.get(...) when accessing parameters
        self.params = params
        # Validate default_covariance once during init. Handlers and publish
        # logic assume this is a length-36 array; fail early if misconfigured.
        default_cov = self.params.get('default_covariance')
        if default_cov is None or len(default_cov) != 36:
            self.get_logger().error(
                "Parameter 'default_covariance' must be a double_array of length 36.")
            raise RuntimeError(
                "invalid parameter: default_covariance must be length 36")
        # Normalize to a plain Python list for downstream use
        self.params['default_covariance'] = list(default_cov)

        # tf2_ros Buffer/Listenerを初期化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom/gps', 10)

        # Handlers: handler is responsible for building 36-element covariance and
        # providing orientation (yaw or quaternion). GNSSOdometryNode performs
        # UTM conversion and places the result into map frame.
        gnss_input = self.params.get('gnss_input')
        if gnss_input == 'navpvt':
            self.handler = NavPVTHandler(self.params)
        else:
            self.handler = NavSatFixHandler(self.params)

        # Unified subscription: get msg_type and topic from handler
        msg_type = self.handler.msg_type
        topic = self.handler.topic

        self.create_subscription(msg_type, topic, self._on_gnss_msg, 10)

        # 暫定: ラベル切替は std_msgs/String トピックで受け付ける
        self.create_subscription(
            String, '~/select_static_transform', self._on_select_label, 10)

        # static_transforms_file を読み込んで初期ラベルを選択
        st_file = self.params.get('static_transforms_file')
        chosen_label = self.params.get('static_transform_label')
        loaded = False
        try:
            loaded = self.transform_manager.load_transforms_from_file(st_file)
        except Exception as e:
            self.get_logger().warn(
                f"Failed to load static transforms file {st_file}: {e}")

        if loaded:
            labels = self.transform_manager.get_labels()
            if not chosen_label and labels:
                chosen_label = labels[0]
            if chosen_label:
                ok = self.transform_manager.set_active_label(chosen_label)
                if ok:
                    self.get_logger().info(
                        f"Active static transform set to label '{chosen_label}' from {st_file}")
                else:
                    self.get_logger().warn(
                        f"Label '{chosen_label}' not found in {st_file}")
        else:
            # 既存の対応点からの推定を試みる（従来の挙動、フォールバック）
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

        self.publish_static_transform()
        self._tested = False

    def _on_select_label(self, msg: String):
        # トピックでラベルを受け取り切替える（暫定実装）
        label = msg.data
        if label is None or label == '':
            self.get_logger().warn('Received empty label on /gnss/select_static_transform')
            return
        ok = self.transform_manager.set_active_label(label)
        if ok:
            self.get_logger().info(
                f"Switched active static transform to label '{label}'")
            self.publish_static_transform()
        else:
            self.get_logger().warn(
                f"Requested label '{label}' not found among available labels: {self.transform_manager.get_labels()}")

    def publish_static_transform(self):
        # static_transform (UTM→map) をtfでpublish
        tf = self.transform_manager.static_transform
        if tf is None or len(tf) != 3:
            return
        x, y, yaw = tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'utm'  # UTM座標系
        # child_frame_id comes from declared parameter 'map_frame_id'
        t.child_frame_id = self.params.get('map_frame_id')
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

    def _on_gnss_msg(self, msg):
        """Unified GNSS message callback. Delegate processing to the handler,
        perform UTM conversion (node responsibility), apply static transform
        to map frame, and publish Odometry using the HandlerResult values.
        """
        if not self._tested:
            self._tested = True
            self.test_transform_accuracy()

        # Let handler process the incoming message and build covariance/orientation
        result = self.handler.process(msg)

        # Convert lat/lon -> UTM (odom_manager expects longitude, latitude)
        try:
            utm_x, utm_y = self.odom_manager.gnss_to_utm(
                result.lon_deg, result.lat_deg)
        except Exception as e:
            self.get_logger().warn(f"Failed to convert GNSS to UTM: {e}")
            return

        # If no correspondences/static transform, optionally log current gps_link position
        if (self.transform_manager.correspondences is None or len(self.transform_manager.correspondences) < 2):
            try:
                target = self.params.get('map_frame_id')
                source = self.params.get('gps_frame_id')
                trans = self.tf_buffer.lookup_transform(target, source, Time())
                map_x_cur = trans.transform.translation.x
                map_y_cur = trans.transform.translation.y
            except Exception:
                self.get_logger().error("Failed to lookup transform for gps_link in map frame.")

        # Convert to map and publish odometry using handler result
        map_x, map_y = self.utm_to_map(utm_x, utm_y)
        self.publish_odometry(map_x, map_y, handler_result=result)

    def publish_odometry(self, map_x, map_y, handler_result: HandlerResult):
        # Build Odometry from HandlerResult (handler owns covariance and orientation)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.params.get('map_frame_id')
        odom.child_frame_id = self.params.get('gps_frame_id')
        odom.pose.pose.position.x = map_x
        odom.pose.pose.position.y = map_y
        odom.pose.pose.position.z = handler_result.alt_m if handler_result.alt_m is not None else 0.0

        # Orientation: prefer quaternion from handler, else yaw
        if handler_result.quaternion is not None:
            odom.pose.pose.orientation = handler_result.quaternion
        elif handler_result.yaw_rad is not None:
            q = tf_transformations.quaternion_from_euler(
                0, 0, handler_result.yaw_rad)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
        else:
            # No orientation available: set neutral quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, 0.0)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

        # Covariance: handler must provide a 36-element covariance. The
        # parameter 'default_covariance' was validated at init; handlers
        # should honor that contract. Use the handler-provided covariance
        # directly (copy to a list to avoid shared-mutable structures).
        odom.pose.covariance = list(handler_result.covariance)
        threshold = 10.0  # [m] 異常に大きな分散はpublishしない
        if odom.pose.covariance[0] + odom.pose.covariance[7] > threshold ** 2:
            self.get_logger().warn(f"cov too large: {odom.pose.covariance}")
            return
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
