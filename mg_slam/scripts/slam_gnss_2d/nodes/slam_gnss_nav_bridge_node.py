#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Quaternion
from ublox_msgs.msg import NavPVT

import pyproj


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
        math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
        math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class SlamGnssNavBridgeNode(Node):
    def __init__(self):
        super().__init__('slam_gnss_nav_bridge_node')

        self.declare_parameter('gnss_transform_file', '')
        # 'navpvt' or 'navsatfix'
        self.declare_parameter('gnss_input', 'navpvt')
        self.declare_parameter('gnss_topic', '/navpvt')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('gps_frame_id', 'gps_link')
        # 'computed' or 'navpvt'
        self.declare_parameter('heading_source', 'computed')
        self.declare_parameter('heading_min_distance', 0.6)
        self.declare_parameter('heading_smoothing_alpha', 0.6)
        self.declare_parameter('min_publish_distance', 1.0)
        self.declare_parameter('max_covariance_threshold', 49.0)

        self._transform_file = self.get_parameter('gnss_transform_file').value
        self._gnss_input = self.get_parameter('gnss_input').value
        self._gnss_topic = self.get_parameter('gnss_topic').value
        self._map_frame_id = self.get_parameter('map_frame_id').value
        self._gps_frame_id = self.get_parameter('gps_frame_id').value
        self._heading_source = self.get_parameter('heading_source').value
        self._heading_min_dist = self.get_parameter(
            'heading_min_distance').value
        self._heading_alpha = self.get_parameter(
            'heading_smoothing_alpha').value
        self._min_pub_dist = self.get_parameter('min_publish_distance').value
        self._max_cov_thresh = self.get_parameter(
            'max_covariance_threshold').value

        self._anchor_lat = 0.0
        self._anchor_lon = 0.0
        self._anchor_easting = 0.0
        self._anchor_northing = 0.0
        self._rotation_rad = 0.0
        self._utm_proj = None

        if not self._load_transform():
            self.get_logger().error("Failed to load GNSS transform. Node will not publish.")
            return

        # setup pyproj
        # EPSG:326xx for North, EPSG:327xx for South
        epsg_code = 32600 + \
            self._utm_zone if self._utm_hemisphere == 'north' else 32700 + self._utm_zone
        self._utm_proj = pyproj.Proj(f"EPSG:{epsg_code}")

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/odom/gps', 10)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._anchor_pub = self.create_publisher(
            NavSatFix, '/slam_gnss_2d/anchor', map_qos)
        self._publish_anchor()

        # Subscribers
        if self._gnss_input == 'navpvt':
            self._gnss_sub = self.create_subscription(
                NavPVT, self._gnss_topic, self._navpvt_callback, 10)
        else:
            self._gnss_sub = self.create_subscription(
                NavSatFix, self._gnss_topic, self._navsatfix_callback, 10)

        self._last_map_x = None
        self._last_map_y = None
        self._last_heading = None
        self._last_pub_x = None
        self._last_pub_y = None

        self.get_logger().info(
            f"SlamGnssNavBridgeNode initialized. Input: {self._gnss_input} ({self._gnss_topic})")

    def _load_transform(self) -> bool:
        if not self._transform_file:
            self.get_logger().error("gnss_transform_file parameter is empty.")
            return False

        try:
            with open(self._transform_file, 'r') as f:
                data = yaml.safe_load(f)

            self._anchor_lat = data['anchor']['latitude']
            self._anchor_lon = data['anchor']['longitude']
            self._anchor_easting = data['anchor_utm']['easting']
            self._anchor_northing = data['anchor_utm']['northing']
            self._utm_zone = data['anchor_utm']['zone']
            self._utm_hemisphere = data['anchor_utm']['hemisphere']
            file_rotation = data['rotation_rad']
            # SLAMマップはすでにUTM座標系にアライメントされて生成されているため、
            # ナビゲーション時の座標変換における追加の回転は不要（0.0）とします。
            self._rotation_rad = 0.0

            self.get_logger().info(
                f"Loaded transform: anchor=({self._anchor_lat}, {self._anchor_lon}), file_rot={file_rotation:.3f}rad, applied_rot={self._rotation_rad:.3f}rad")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to read transform file {self._transform_file}: {e}")
            return False

    def _publish_anchor(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame_id
        msg.latitude = float(self._anchor_lat)
        msg.longitude = float(self._anchor_lon)
        self._anchor_pub.publish(msg)
        self.get_logger().info(
            f"Published anchor: lat={msg.latitude}, lon={msg.longitude}")

    def _convert_to_map(self, lat, lon):
        utm_e, utm_n = self._utm_proj(lon, lat)

        local_x = utm_e - self._anchor_easting
        local_y = utm_n - self._anchor_northing

        cos_r = math.cos(self._rotation_rad)
        sin_r = math.sin(self._rotation_rad)

        map_x = cos_r * local_x - sin_r * local_y
        map_y = sin_r * local_x + cos_r * local_y

        return map_x, map_y

    def _compute_heading(self, map_x, map_y):
        if self._last_map_x is None:
            self._last_map_x = map_x
            self._last_map_y = map_y
            return None

        dx = map_x - self._last_map_x
        dy = map_y - self._last_map_y
        dist = math.hypot(dx, dy)

        if dist >= self._heading_min_dist:
            new_heading = math.atan2(dy, dx)
            if self._last_heading is None:
                self._last_heading = new_heading
            else:
                # Circular EMA
                diff = (new_heading - self._last_heading +
                        math.pi) % (2 * math.pi) - math.pi
                self._last_heading += self._heading_alpha * diff
                self._last_heading = (
                    self._last_heading + math.pi) % (2 * math.pi) - math.pi

            self._last_map_x = map_x
            self._last_map_y = map_y

        return self._last_heading

    def _navpvt_callback(self, msg: NavPVT):
        self.get_logger().debug(
            f"Received NavPVT: lat={msg.lat * 1e-7:.7f}, lon={msg.lon * 1e-7:.7f}, hAcc={msg.h_acc}mm, flags={msg.flags}")
        # basic validation
        if msg.flags & 1 == 0:  # gnssFixOK is bit 0
            self.get_logger().warn("NavPVT does not have a valid fix. Skipping.")
            return

        # hAcc is in mm, convert to meters
        hacc = msg.h_acc / 1000.0
        cov_xx = hacc * hacc

        if cov_xx * 2 > self._max_cov_thresh:
            return

        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7

        map_x, map_y = self._convert_to_map(lat, lon)

        if self._last_pub_x is not None:
            pub_dist = math.hypot(map_x - self._last_pub_x,
                                  map_y - self._last_pub_y)
            if pub_dist < self._min_pub_dist:
                return

        heading = None
        if self._heading_source == 'navpvt':
            # msg.heading is heading of motion (2-D) in 1e-5 degrees
            heading_deg = msg.heading * 1e-5
            heading = math.radians(heading_deg)
            # Need to apply rotation offset
            heading += self._rotation_rad
            # ENU coordinate system
            heading = -heading + (math.pi / 2.0)
            heading = (heading + math.pi) % (2 * math.pi) - math.pi
        else:
            heading = self._compute_heading(map_x, map_y)

        if heading is None:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._gps_frame_id

        self._publish_odom(header, map_x, map_y, heading, cov_xx)
        self._last_pub_x = map_x
        self._last_pub_y = map_y

    def _navsatfix_callback(self, msg: NavSatFix):
        if msg.status.status < 0:  # NO_FIX
            return

        cov_xx = msg.position_covariance[0]
        if cov_xx * 2 > self._max_cov_thresh:
            return

        map_x, map_y = self._convert_to_map(msg.latitude, msg.longitude)

        if self._last_pub_x is not None:
            pub_dist = math.hypot(map_x - self._last_pub_x,
                                  map_y - self._last_pub_y)
            if pub_dist < self._min_pub_dist:
                return

        heading = self._compute_heading(map_x, map_y)
        if heading is None:
            return

        self._publish_odom(msg.header, map_x, map_y, heading, cov_xx)
        self._last_pub_x = map_x
        self._last_pub_y = map_y

    def _publish_odom(self, header, x, y, yaw, pos_cov):
        odom = Odometry()
        odom.header.stamp = header.stamp
        # Re-assigning to avoid NavPVT vs standard header discrepancies
        odom.header.frame_id = self._map_frame_id
        odom.child_frame_id = self._gps_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = euler_to_quaternion(0, 0, yaw)

        # Set covariance (x, y, z, r, p, y)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = pos_cov
        odom.pose.covariance[7] = pos_cov
        odom.pose.covariance[14] = 99999.0
        odom.pose.covariance[21] = 99999.0
        odom.pose.covariance[28] = 99999.0
        odom.pose.covariance[35] = 0.1  # yaw covariance

        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SlamGnssNavBridgeNode()
    if node._utm_proj is not None:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
