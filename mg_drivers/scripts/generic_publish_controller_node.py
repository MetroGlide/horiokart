#!/usr/bin/env python3

import importlib
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class GenericPublishControllerNode(Node):
    """汎用のトピック配信制御ノード

    - 動的にメッセージ型をインポート（`msg_module`, `msg_class` パラメータ）
    - `publish` パラメータで初期の配信状態を決定
    - トピック名・サービス名はハードコーディング（launch で remap 想定）
    """

    def __init__(self):
        super().__init__("generic_publish_controller_node")

        self.get_logger().info("generic_publish_controller_node started")

        # 初期値は init_parameters 内でセットされる
        self._publish = True
        self._queue_size = 1
        self._msg_type = None

        self.init_parameters()
        self.init_ros_topics()

        self.get_logger().info("generic_publish_controller_node initialized")

    def init_parameters(self):
        # パラメータ宣言
        # - publish: 起動時の配信有無
        # - msg_module, msg_class: 動的に読み込むメッセージ型（必須）
        # - queue_size: サブスク/パブリッシャのキューサイズ
        self.declare_parameter('publish', True)
        # デフォルトは空文字列にして必須チェックを行う
        self.declare_parameter('msg_module', '')
        self.declare_parameter('msg_class', '')
        self.declare_parameter('queue_size', 1)

        # パラメータ取得（declare_parameter でデフォルトが保証されるため直接取得）
        self._publish = bool(self.get_parameter(
            'publish').get_parameter_value().bool_value)

        self._queue_size = int(self.get_parameter(
            'queue_size').get_parameter_value().integer_value)
        if self._queue_size <= 0:
            self.get_logger().warning('Parameter "queue_size" must be > 0; default 1 used')
            self._queue_size = 1

        # msg_module / msg_class は必須とする
        self._msg_module = str(self.get_parameter(
            'msg_module').get_parameter_value().string_value)
        self._msg_class = str(self.get_parameter(
            'msg_class').get_parameter_value().string_value)

        if not self._msg_module or not self._msg_class:
            self.get_logger().error('Parameters "msg_module" and "msg_class" must be set (non-empty)')
            rclpy.shutdown()
            sys.exit(1)

        # 動的インポート
        try:
            module = importlib.import_module(self._msg_module)
        except Exception as e:
            self.get_logger().error(
                f'Failed to import message module "{self._msg_module}": {e}')
            rclpy.shutdown()
            sys.exit(1)

        try:
            self._msg_type = getattr(module, self._msg_class)
        except AttributeError:
            self.get_logger().error(
                f'Message class "{self._msg_class}" not found in module "{self._msg_module}"'
            )
            rclpy.shutdown()
            sys.exit(1)

        # 簡易チェック
        if not hasattr(self._msg_type, '__slots__'):
            self.get_logger().warning(
                f'Imported object {self._msg_class} from {self._msg_module} may not be a ROS message type'
            )

    def init_ros_topics(self):
        # メッセージ型が取得できていることを保証
        if self._msg_type is None:
            self.get_logger().error('Message type not available; cannot create topics')
            rclpy.shutdown()
            sys.exit(1)

        input_topic = 'input_topic'
        output_topic = 'output_topic'
        service_name = '~/change_publish_state'

        self._subscriber = self.create_subscription(
            self._msg_type,
            input_topic,
            self._on_message,
            self._queue_size,
        )

        self._publisher = self.create_publisher(
            self._msg_type,
            output_topic,
            self._queue_size,
        )

        self._change_publish_state_service = self.create_service(
            SetBool,
            service_name,
            self._change_publish_state_callback,
        )

        self.get_logger().info(
            f'Created subscription({input_topic}) and publisher({output_topic}) for {self._msg_module}.{self._msg_class}'
        )

    def _on_message(self, msg):
        if self._publish:
            self._publisher.publish(msg)

    def _change_publish_state_callback(self, req, res):
        self._publish = bool(req.data)
        res.success = True
        res.message = 'success'
        return res


if __name__ == '__main__':
    rclpy.init()
    node = GenericPublishControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
