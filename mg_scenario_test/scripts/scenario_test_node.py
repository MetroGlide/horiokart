#!/usr/bin/env python3
import json
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from mg_scenario_test.adapters.gazebo import GazeboAdapter
from mg_scenario_test.event_executor import EventExecutor
from mg_scenario_test.scenario_loader import ScenarioLoader
from mg_scenario_test.scenario_runner import ScenarioRunner


class ScenarioTestNode(Node):
    def __init__(self):
        super().__init__("scenario_test_node")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("robot_name", "mg")

        self._result_pub = self.create_publisher(String, "~/result", 10)

        scenario_file = self.get_parameter("scenario_file").value
        if not scenario_file:
            self.get_logger().fatal("Parameter 'scenario_file' is required")
            raise RuntimeError("scenario_file not specified")

        self._scenario = ScenarioLoader.load(scenario_file)
        self.get_logger().info(
            f"Loaded scenario '{self._scenario.scenario_name}' from {scenario_file}"
        )

        robot_name = self.get_parameter("robot_name").value
        adapter = GazeboAdapter(
            world_name=self._scenario.world_name,
            robot_name=robot_name,
        )
        event_executor = EventExecutor(
            node=self,
            adapter=adapter,
            obstacles=self._scenario.obstacles,
        )
        self._runner = ScenarioRunner(
            node=self,
            adapter=adapter,
            event_executor=event_executor,
        )

        self._exec_thread = threading.Thread(
            target=self._run_scenario, daemon=True
        )
        self._exec_thread.start()

    def _run_scenario(self) -> None:
        self.get_logger().info(
            f"[ScenarioTest] Starting scenario '{self._scenario.scenario_name}'"
        )
        result = self._runner.execute(self._scenario)

        payload = {
            "scenario_name": self._scenario.scenario_name,
            "success": result.success,
            "elapsed_sec": round(result.elapsed_sec, 2),
            "reached_count": result.reached_count,
            "total_count": result.total_count,
            "failed_index": result.failed_index,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._result_pub.publish(msg)

        if result.success:
            self.get_logger().info(
                f"[ScenarioTest] PASSED — {result.reached_count}/{result.total_count} goals, "
                f"{result.elapsed_sec:.1f}s"
            )
        else:
            self.get_logger().error(
                f"[ScenarioTest] FAILED at goal {result.failed_index} — "
                f"{result.reached_count}/{result.total_count} goals, "
                f"{result.elapsed_sec:.1f}s"
            )


def main():
    rclpy.init()
    node = ScenarioTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
