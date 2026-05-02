#!/usr/bin/env python3
import json
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from mg_scenario_test.adapters.gazebo import GazeboAdapter
from mg_scenario_test.event_executor import EventExecutor
from mg_scenario_test.scenario_loader import ScenarioLoader
from mg_scenario_test.scenario_runner import ScenarioRunner

_SEP = "=" * 60


class ScenarioTestNode(Node):
    def __init__(self):
        super().__init__("scenario_test_node")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("robot_name", "mg")

        self._result_pub = self.create_publisher(String, "~/result", 10)
        self._done_event = threading.Event()

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

        log = self.get_logger()
        log.info(_SEP)
        if result.success:
            log.info("  Result : PASSED")
        else:
            log.error("  Result : FAILED")
        log.info(
            f"  Goals  : {result.reached_count} / {result.total_count} reached")
        log.info(f"  Time   : {result.elapsed_sec:.1f} s")
        if not result.success and result.failed_index >= 0:
            log.error(f"  Failed at goal index: {result.failed_index}")
        log.info(_SEP)

        self._done_event.set()

    def wait_for_done(self) -> bool:
        """シナリオ完了まで待機して結果を返す。"""
        self._exec_thread.join()
        return True


def main():
    rclpy.init()
    node = ScenarioTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node._done_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
