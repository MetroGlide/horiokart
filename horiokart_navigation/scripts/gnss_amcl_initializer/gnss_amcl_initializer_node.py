#!/usr/bin/env python3

"""GNSS -> AMCL initializer.

Subscribe to `/odom/gps` (nav_msgs/Odometry) containing GNSS-derived
pose in the map frame. When a sequence of odometry messages meets the
configured accuracy thresholds, publish a PoseWithCovarianceStamped to
initialize AMCL. A node-private service is provided to reset and retry
the initialization logic.
"""

import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_srvs.srv import Trigger

import tf_transformations
import tf2_ros
from tf2_ros import TransformException


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat


class GNSSQualityManager:
    """Track consecutive-good / consecutive-bad counts and finished state.

    Responsibilities:
    - maintain consecutive_good / consecutive_bad counters
    - decide when to mark finished (publish reached) or give up
    - provide simple query methods for callers (is_publish_ready/is_gave_up)

    The owning node keeps the latest Odometry message; this class only
    tracks numeric state and optional logging.
    """

    def __init__(self, required_consecutive_good: int, max_consecutive_bad: int, logger=None):
        self.required_consecutive_good = int(required_consecutive_good)
        self.max_consecutive_bad = int(max_consecutive_bad)
        self.consecutive_good = 0
        self.consecutive_bad = 0
        self.finished = False
        self._logger = logger

    def reset(self) -> None:
        """Reset counters and allow processing to continue."""
        self.consecutive_good = 0
        self.consecutive_bad = 0
        self.finished = False

    def add_bad(self) -> None:
        """Record a bad sample and mark give-up if threshold exceeded."""
        self.consecutive_good = 0
        self.consecutive_bad += 1
        if self.consecutive_bad >= self.max_consecutive_bad:
            self.finished = True
            if self._logger:
                self._logger.warn(
                    'Too many consecutive bad odom samples; giving up')

    def add_good(self) -> None:
        """Record a good sample (increments counter and clears bad counter)."""
        self.consecutive_bad = 0
        self.consecutive_good += 1
        if self.consecutive_good >= self.required_consecutive_good:
            self.finished = True
            if self._logger:
                self._logger.info(
                    'Required consecutive good odom samples reached')

    def is_publish_ready(self) -> bool:
        """Return True when enough consecutive good samples have been seen."""
        return self.consecutive_good >= self.required_consecutive_good

    def is_gave_up(self) -> bool:
        """Return True if the state has given up due to too many bad samples."""
        return self.finished and (self.consecutive_bad >= self.max_consecutive_bad)

    def mark_published(self) -> None:
        """Mark that a publish has occurred (keeps finished True but resets
        the consecutive_good counter to allow reinit-based reprocessing).
        """
        self.finished = True
        self.consecutive_good = 0


class GNSSAMCLInitializer(Node):
    def __init__(self):
        super().__init__('gnss_amcl_initializer')
        # Initialize parameters and node configuration
        self._init_parameters()

        # Internal state is managed by GNSSQualityManager to keep good/bad logic
        # together and simplify reasoning about finished/reset behavior.
        self.state = GNSSQualityManager(self.required_consecutive_good,
                                        self.max_consecutive_bad,
                                        logger=self.get_logger())
        # The node retains ownership of the latest valid odometry sample.
        self.latest_valid_odom: Optional[Odometry] = None

        # Initialize TF, publishers/subscribers and services
        self._init_communications()

        self.get_logger().info(
            f"GNSS AMCL Initializer started, listening to '{self.odom_gps_topic}'")

    # ---------------- callbacks ----------------

    def odom_callback(self, msg: Odometry) -> None:
        # If finished (published or gave up), ignore further samples.
        if self.state.finished:
            return

    # Age check (can be disabled via ignore_odom_age)
        try:
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        except Exception:
            # malformed header; treat as invalid
            self.state.add_bad()
            return

        if not self.ignore_odom_age and age > self.odom_age_timeout_sec:
            # aged message -> treat as bad
            self.state.add_bad()
            return

    # Evaluate odometry quality (covariance thresholds)
        if not self._evaluate_odometry_quality(msg):
            # _evaluate_odometry_quality logs reason
            self.state.add_bad()
            return

    # Passed quality checks: transform to map frame if needed and log
        try:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            self.get_logger().info(
                f"Received odom: frame={msg.header.frame_id}, pos=({px:.3f},{py:.3f},{pz:.3f}), quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
        except Exception:
            self.get_logger().debug('Received odom: unable to extract full summary')

    # Transform pose into map frame if required
        odom_in_map = msg
        if msg.header.frame_id != self.map_frame:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id, Time())
                odom_in_map = self._transform_odometry_pose(msg, trans)
                self.get_logger().debug('TF transform to map succeeded')
            except TransformException:
                # fall back to original odom if TF fails
                pass

    # Good sample: store latest odom, update state, and publish when ready.
        self.latest_valid_odom = odom_in_map
        self.state.add_good()
        self.get_logger().info(
            f'Good odom #{self.state.consecutive_good}/{self.required_consecutive_good} (pos=({odom_in_map.pose.pose.position.x:.3f},{odom_in_map.pose.pose.position.y:.3f}))')

        if self.state.is_publish_ready():
            self.get_logger().info('Publishing initialpose based on GNSS odometry')
            self.publish_initialpose_from_odom(self.latest_valid_odom)
            # mark published/gone-to-finished and reset per-policy
            self.state.mark_published()

    # ---------------- core utilities ----------------
    def _evaluate_odometry_quality(self, odom: Odometry) -> bool:
        # Expect 6x6 covariance list
        cov = odom.pose.covariance
        if cov is None or len(cov) != 36:
            self.get_logger().info('Odometry covariance missing or malformed')
            return False

        var_x = float(cov[0]) * float(self.covariance_scale)
        var_y = float(cov[7]) * float(self.covariance_scale)
        var_z = float(cov[14]) * float(self.covariance_scale)

        std_x = math.sqrt(max(var_x, 0.0))
        std_y = math.sqrt(max(var_y, 0.0))
        std_z = math.sqrt(max(var_z, 0.0))

        ok_xy = (std_x <= self.max_position_std_m) and (
            std_y <= self.max_position_std_m)
        ok_z = std_z <= self.max_vertical_std_m

        self.get_logger().info(
            f'Odom std (x,y,z)=({std_x:.3f},{std_y:.3f},{std_z:.3f}), thresholds (xy,z)=({self.max_position_std_m},{self.max_vertical_std_m})')
        if ok_xy and ok_z:
            self.get_logger().info('Odometry judged GOOD')
        else:
            self.get_logger().info('Odometry judged BAD')
        return ok_xy and ok_z

    # ---------------- init helpers ----------------
    def _init_parameters(self) -> None:
        """Declare and read parameters used by the node."""
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('required_consecutive_good', 5)
        self.declare_parameter('max_position_std_m', 5.0)
        self.declare_parameter('max_vertical_std_m', 10.0)
        self.declare_parameter('use_fixed_heading', True)
        self.declare_parameter('fixed_heading', 1.57)
        self.declare_parameter('covariance_scale', 1.0)
        # orientation_covariance: [roll_var, pitch_var, yaw_var]
        self.declare_parameter('orientation_covariance',
                               [9999.0, 9999.0, 9999.0])
        # allow overriding the full 6x6 pose covariance via parameter
        self.declare_parameter('override_pose_covariance', False)
        # expected length 36 (row-major 6x6). Default: zeros
        self.declare_parameter('pose_covariance', [0.0] * 36)
        self.declare_parameter('odom_age_timeout_sec', 2.0)
        self.declare_parameter('ignore_odom_age', False)
        self.declare_parameter('max_consecutive_bad', 20)

        # Read other tunable parameters
        self.map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter(
            'base_link_frame').get_parameter_value().string_value
        self.required_consecutive_good = self.get_parameter(
            'required_consecutive_good').get_parameter_value().integer_value
        self.max_position_std_m = self.get_parameter(
            'max_position_std_m').get_parameter_value().double_value
        self.max_vertical_std_m = self.get_parameter(
            'max_vertical_std_m').get_parameter_value().double_value
        self.use_fixed_heading = self.get_parameter(
            'use_fixed_heading').get_parameter_value().bool_value
        self.fixed_heading = self.get_parameter(
            'fixed_heading').get_parameter_value().double_value
        self.covariance_scale = self.get_parameter(
            'covariance_scale').get_parameter_value().double_value
        self.orientation_covariance = self.get_parameter(
            'orientation_covariance').get_parameter_value().double_array_value
        self.override_pose_covariance = self.get_parameter(
            'override_pose_covariance').get_parameter_value().bool_value
        self.pose_covariance = self.get_parameter(
            'pose_covariance').get_parameter_value().double_array_value
        self.odom_age_timeout_sec = self.get_parameter(
            'odom_age_timeout_sec').get_parameter_value().double_value
        self.ignore_odom_age = self.get_parameter(
            'ignore_odom_age').get_parameter_value().bool_value
        self.max_consecutive_bad = self.get_parameter(
            'max_consecutive_bad').get_parameter_value().integer_value

        self.get_logger().info(
            f"Parameters: map_frame={self.map_frame}, required_consecutive_good={self.required_consecutive_good}")
        self.get_logger().info(
            f"Thresholds: max_position_std_m={self.max_position_std_m}, max_vertical_std_m={self.max_vertical_std_m}, covariance_scale={self.covariance_scale}")
        self.get_logger().info(
            f"Heading: use_fixed_heading={self.use_fixed_heading}, fixed_heading={self.fixed_heading}")
        self.get_logger().info(
            f"Pose covariance override: override_pose_covariance={self.override_pose_covariance}")
        try:
            use_sim = self.get_parameter(
                'use_sim_time').get_parameter_value().bool_value
            self.get_logger().info(f'use_sim_time={use_sim}')
        except Exception:
            pass
        if self.ignore_odom_age:
            self.get_logger().info('Odom age check is DISABLED (ignore_odom_age=True)')
        self.get_logger().info(
            f'max_consecutive_bad={self.max_consecutive_bad}')

    def _init_communications(self) -> None:
        """Initialize TF, publishers, subscribers and services."""
        # TF buffer/listener for optional frame transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self.odom_gps_topic = '/odom/gps'
        self.initialpose_topic = '/initialpose'
        # node-private service name for reinit requests; remap/namespace can
        # be applied from the outside launch file. Use a descriptive name.
        self.reinit_service_name = '~/request_reinit'

        # Log parameter summary for debugging
        self.get_logger().info(
            f"Topics/Services: odom_gps_topic={self.odom_gps_topic}, initialpose_topic={self.initialpose_topic}, reinit_service={self.reinit_service_name}")

        # Publisher and subscribers
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_gps_topic, self.odom_callback, 20)

        # Service to force reinit (resets internal state so processing can resume)
        self.srv = self.create_service(
            Trigger, self.reinit_service_name, self.handle_reinit)

    def _transform_odometry_pose(self, odom: Odometry, trans) -> Odometry:
        # Create a shallow copy of odom with transformed pose into map_frame
        new = Odometry()
        new.header = odom.header
        new.header.frame_id = self.map_frame
        new.child_frame_id = odom.child_frame_id
        # apply transform: map_pose = transform * odom.pose.pose
        # transform has translation (x,y,z) and rotation quaternion
        # We'll convert both to matrices and compose
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        tq = trans.transform.rotation
        rot = tf_transformations.quaternion_matrix([tq.x, tq.y, tq.z, tq.w])
        pt = [px, py, pz, 1.0]
        mapped = rot.dot(pt)
        new.pose.pose.position.x = mapped[0] + tx
        new.pose.pose.position.y = mapped[1] + ty
        new.pose.pose.position.z = mapped[2] + tz

        # rotate orientation
        q_in = odom.pose.pose.orientation
        q_in_list = [q_in.x, q_in.y, q_in.z, q_in.w]
        q_map = tf_transformations.quaternion_multiply(
            [tq.x, tq.y, tq.z, tq.w], q_in_list)
        new.pose.pose.orientation.x = q_map[0]
        new.pose.pose.orientation.y = q_map[1]
        new.pose.pose.orientation.z = q_map[2]
        new.pose.pose.orientation.w = q_map[3]

        # copy covariance (no change in this simple transform approximation)
        new.pose.covariance = list(odom.pose.covariance)
        return new

    def publish_initialpose_from_odom(self, odom: Optional[Odometry]) -> None:
        if odom is None:
            self.get_logger().warn('No valid odometry available for initialpose')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        # position
        msg.pose.pose.position.x = odom.pose.pose.position.x
        msg.pose.pose.position.y = odom.pose.pose.position.y
        msg.pose.pose.position.z = odom.pose.pose.position.z

        # orientation: prefer heading contained in Odometry; if not available use fixed parameter when enabled
        q = odom.pose.pose.orientation
        yaw = None
        has_orientation = not (
            abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6 and abs(q.w - 1.0) < 1e-6
        )
        if has_orientation:
            try:
                yaw = tf_transformations.euler_from_quaternion([
                    q.x, q.y, q.z, q.w
                ])[2]
            except Exception:
                yaw = None
        if yaw is None and self.use_fixed_heading:
            yaw = float(self.fixed_heading)
        if yaw is None:
            yaw = 0.0

        msg.pose.pose.orientation = quaternion_from_yaw(
            yaw if yaw is not None else 0.0)

        # If operator requested overriding the full pose covariance via parameter,
        # use that 6x6 (36-element row-major) array directly.
        if getattr(self, 'override_pose_covariance', False):
            try:
                if self.pose_covariance and len(self.pose_covariance) == 36:
                    msg.pose.covariance = [float(v)
                                           for v in self.pose_covariance]
                    self.initialpose_pub.publish(msg)
                    self.get_logger().info(
                        'Published initialpose using parameter override for pose_covariance')
                    self.get_logger().info(
                        f'Initialpose covariance: {msg.pose.covariance}')
                    return
                else:
                    self.get_logger().warn(
                        'override_pose_covariance is True but pose_covariance param length != 36; ignoring override')
            except Exception:
                self.get_logger().warn(
                    'Error applying pose_covariance override; ignoring and falling back to odom covariance')

        # covariance mapping: odom.pose.covariance (6x6 row-major) -> pose.covariance (6x6)
        cov = list(odom.pose.covariance) if odom.pose.covariance is not None and len(
            odom.pose.covariance) == 36 else [0.0]*36
        # scale position covariances
        cov[0] = cov[0] * self.covariance_scale
        cov[1] = cov[1] * self.covariance_scale
        cov[2] = cov[2] * self.covariance_scale
        cov[3] = cov[3] * self.covariance_scale
        cov[4] = cov[4] * self.covariance_scale
        cov[5] = cov[5] * self.covariance_scale
        cov[6] = cov[6] * self.covariance_scale
        cov[7] = cov[7] * self.covariance_scale
        cov[8] = cov[8] * self.covariance_scale

        # orientation covariance override from parameter: roll_var, pitch_var, yaw_var
        roll_var, pitch_var, yaw_var = (float(v) for v in (
            self.orientation_covariance or [9999.0, 9999.0, 9999.0]))

        # Compose final 6x6 covariance
        final_cov = [0.0] * 36
        # position block
        final_cov[0] = cov[0]
        final_cov[1] = cov[1]
        final_cov[2] = cov[2]
        final_cov[6] = cov[6]
        final_cov[7] = cov[7]
        final_cov[8] = cov[8]
        final_cov[12] = cov[12] if len(cov) > 12 else 0.0
        final_cov[13] = cov[13] if len(cov) > 13 else 0.0
        final_cov[14] = cov[14] if len(cov) > 14 else 0.0

        # orientation block (rows/cols 3..5)
        final_cov[21] = roll_var
        final_cov[28] = pitch_var
        final_cov[35] = yaw_var

        msg.pose.covariance = final_cov

        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'Published initialpose at ({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}), yaw={yaw:.3f}')
        self.get_logger().info(
            f'Initialpose covariance: {msg.pose.covariance}')

    # ---------------- service handlers ----------------
    def handle_reinit(self, request, response):
        # Service to request re-initialization: reset internal state so the
        # node can re-process incoming odometry samples. Per request,
        # this service only resets state and always returns success=True.
        self.get_logger().info('Reinitialize request received; resetting internal state')

        # Reset processing state; do not attempt an immediate publish here.
        self.state.reset()
        # Clear stored latest odom so re-processing starts fresh
        self.latest_valid_odom = None

        response.success = True
        response.message = 'Reinitialization requested'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GNSSAMCLInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
