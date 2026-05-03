#!/usr/bin/env python3
import subprocess
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mg_msgs.srv import ResetSimRobotPose


class SystemManagerNode(Node):
    """make / docker compose コマンドをROS2サービス経由で実行する。"""

    def __init__(self):
        super().__init__('system_manager_node')

        self.declare_parameter('project_dir', '/app')
        self.declare_parameter('simulation_world', 'warehouse')
        self.declare_parameter('simulation_robot_name', 'mg')
        self._project_dir: str = (
            self.get_parameter(
                'project_dir').get_parameter_value().string_value
        )
        self._simulation_world: str = (
            self.get_parameter(
                'simulation_world').get_parameter_value().string_value
        )
        self._simulation_robot_name: str = (
            self.get_parameter(
                'simulation_robot_name').get_parameter_value().string_value
        )

        self.create_service(Trigger, '~/start_slam', self._start_slam)
        self.create_service(Trigger, '~/stop_slam', self._stop_slam)
        self.create_service(Trigger, '~/start_navigation',
                            self._start_navigation)
        self.create_service(Trigger, '~/stop_navigation',
                            self._stop_navigation)
        self.create_service(Trigger, '~/save_map', self._save_map)
        self.create_service(Trigger, '~/start_waypoint_editor',
                            self._start_waypoint_editor)
        self.create_service(Trigger, '~/start_scenario_test',
                            self._start_scenario_test)
        self.create_service(Trigger, '~/stop_scenario_test',
                            self._stop_scenario_test)
        self.create_service(ResetSimRobotPose, '~/reset_sim_robot_pose',
                            self._reset_sim_robot_pose)

        self._status_pub = self.create_publisher(
            String, '~/container_status', 10)
        self.create_timer(2.0, self._publish_container_status)

        self.get_logger().info('system_manager_node started')

    def _run(self, cmd: list[str]) -> tuple[bool, str]:
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
                cwd=self._project_dir,
            )
            if result.returncode == 0:
                return True, result.stdout.strip()
            return False, result.stderr.strip()
        except subprocess.TimeoutExpired:
            return False, 'command timed out'
        except Exception as e:
            return False, str(e)

    def _start_slam(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run(['make', 'slam', 'DETACH=1'])
        res.success = ok
        res.message = msg
        return res

    def _stop_slam(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run(['docker', 'compose', 'stop', 'slam'])
        res.success = ok
        res.message = msg
        return res

    def _start_navigation(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run(['make', 'navigation', 'DETACH=1'])
        res.success = ok
        res.message = msg
        return res

    def _stop_navigation(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run(['docker', 'compose', 'stop', 'navigation'])
        res.success = ok
        res.message = msg
        return res

    def _save_map(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run([
            'docker', 'compose', 'exec', 'slam',
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source /root/ros2_ws/install/setup.bash && '
            'ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap '
            '"{map_topic: map, map_url: /root/ros2_data/map, image_format: pgm, '
            'map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"',
        ])
        res.success = ok
        res.message = msg
        return res

    def _start_waypoint_editor(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run([
            'docker', 'compose', 'exec', '-d', 'develop',
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source /root/ros2_ws/install/setup.bash && '
            'ros2 run mg_waypoint_navigation waypoint_editor_node.py',
        ])
        res.success = ok
        res.message = msg
        return res

    def _start_scenario_test(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run(['make', 'scenario-test', 'DETACH=1'])
        res.success = ok
        res.message = msg
        return res

    def _stop_scenario_test(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok, msg = self._run(['docker', 'compose', 'stop', 'scenario-test'])
        res.success = ok
        res.message = msg
        return res

    def _reset_sim_robot_pose(self, req: ResetSimRobotPose.Request, res: ResetSimRobotPose.Response) -> ResetSimRobotPose.Response:
        qz = math.sin(req.yaw / 2.0)
        qw = math.cos(req.yaw / 2.0)
        request = (
            f'name: "{self._simulation_robot_name}" '
            f'position {{ x: {req.x} y: {req.y} z: {req.z} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz} w: {qw} }}'
        )
        command = (
            f'ign service '
            f'-s /world/{self._simulation_world}/set_pose '
            f'--reqtype ignition.msgs.Pose '
            f'--reptype ignition.msgs.Boolean '
            f'--timeout 5000 '
            f"--req '{request}'"
        )
        ok, msg = self._run([
            'docker', 'compose', 'exec', 'gazebo-simulation',
            'bash', '-lc',
            command,
        ])
        res.success = ok
        res.message = msg
        return res

    def _publish_container_status(self) -> None:
        ok, output = self._run([
            'docker', 'compose', 'ps', '--format', 'json',
        ])
        if not ok:
            return

        services: dict[str, str] = {}
        for line in output.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
                name = entry.get('Service', entry.get('Name', ''))
                state = entry.get('State', 'unknown')
                services[name] = state
            except json.JSONDecodeError:
                continue

        msg = String()
        msg.data = json.dumps(services)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
