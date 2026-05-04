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
        self.get_logger().info('[Called] start_slam')

        ok, msg = self._run(['make', 'slam', 'DETACH=1'])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'start_slam result: success={ok}, message="{msg}"')
        return res

    def _stop_slam(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] stop_slam')

        ok, msg = self._run(['docker', 'compose', 'stop', 'slam'])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'stop_slam result: success={ok}, message="{msg}"')
        return res

    def _start_navigation(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] start_navigation')

        ok, msg = self._run(['make', 'navigation', 'DETACH=1'])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'start_navigation result: success={ok}, message="{msg}"')
        return res

    def _stop_navigation(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] stop_navigation')

        ok, msg = self._run(['docker', 'compose', 'stop', 'navigation'])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'stop_navigation result: success={ok}, message="{msg}"')
        return res

    def _save_map(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] save_map')

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

        self.get_logger().info(
            f'save_map result: success={ok}, message="{msg}"')
        return res

    def _start_waypoint_editor(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] start_waypoint_editor')

        ok, msg = self._run([
            'docker', 'compose', 'exec', '-d', 'develop',
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source /root/ros2_ws/install/setup.bash && '
            'ros2 run mg_waypoint_navigation waypoint_editor_node.py',
        ])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'start_waypoint_editor result: success={ok}, message="{msg}"')
        return res

    def _start_scenario_test(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] start_scenario_test')

        ok, msg = self._run(['make', 'scenario-test', 'DETACH=1'])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'start_scenario_test result: success={ok}, message="{msg}"')
        return res

    def _stop_scenario_test(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[Called] stop_scenario_test')

        ok, msg = self._run(['docker', 'compose', 'stop', 'scenario-test'])
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'stop_scenario_test result: success={ok}, message="{msg}"')
        return res

    def _reset_sim_robot_pose(self, req: ResetSimRobotPose.Request, res: ResetSimRobotPose.Response) -> ResetSimRobotPose.Response:
        self.get_logger().info('[Called] reset_sim_robot_pose')

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

        ok, cid_output = self._run([
            'docker', 'ps',
            '--filter', 'label=com.docker.compose.service=gazebo-simulation',
            '--format', '{{.ID}}'
        ])

        if not ok or not cid_output.strip():
            res.success = False
            res.message = f"Failed to get container ID: {cid_output}"

            self.get_logger().info(
                f'reset_sim_robot_pose result: success={res.success}, message="{res.message}"')
            return res

        # 複数行返ってきた場合に備え、先頭のコンテナIDのみを抽出
        cid = cid_output.strip().split('\n')[0]

        ok, msg = self._run([
            'docker', 'exec', cid.strip(),
            'bash', '-lc',
            command,
        ])
        self.get_logger().info(f'Command output: {msg}')
        res.success = ok
        res.message = msg

        self.get_logger().info(
            f'reset_sim_robot_pose result: success={res.success}, message="{res.message}"')
        return res

    def _publish_container_status(self) -> None:
        self.get_logger().info('[Called] publish_container_status')

        # docker compose ps のバグを回避し、純粋な docker ps を使用する
        # -a をつけることで停止中のコンテナステータスも取得します
        ok, output = self._run([
            'docker', 'ps', '-a',
            '--filter', 'label=com.docker.compose.service',
            '--format', '{{.Label "com.docker.compose.service"}}\t{{.State}}'
        ])

        if not ok:
            self.get_logger().error(
                f'Failed to get container status: {output}')
            return

        services: dict[str, str] = {}
        for line in output.splitlines():
            line = line.strip()
            if not line:
                continue

            # タブ区切りで分割 (例: "gazebo-simulation\trunning")
            parts = line.split('\t')
            if len(parts) >= 2:
                name = parts[0]
                state = parts[1]
                services[name] = state

        msg = String()
        msg.data = json.dumps(services)
        self.get_logger().info(f'Publishing container status: {msg.data}')
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
