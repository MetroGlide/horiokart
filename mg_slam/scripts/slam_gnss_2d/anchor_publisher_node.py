#!/usr/bin/env python3
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix

class AnchorPublisherNode(Node):
    def __init__(self):
        super().__init__('anchor_publisher_node')
        
        self.declare_parameter('gnss_transform_file', '')
        self.declare_parameter('map_frame_id', 'map')
        
        self._transform_file = self.get_parameter('gnss_transform_file').value
        self._map_frame_id = self.get_parameter('map_frame_id').value
        
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(NavSatFix, '/slam_gnss_2d/anchor', map_qos)
        
        if not self._transform_file:
            self.get_logger().error("gnss_transform_file parameter is empty.")
            return
            
        try:
            with open(self._transform_file, 'r') as f:
                data = yaml.safe_load(f)
                
            lat = data['anchor']['latitude']
            lon = data['anchor']['longitude']
            
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame_id
            msg.latitude = float(lat)
            msg.longitude = float(lon)
            
            self._pub.publish(msg)
            self.get_logger().info(f"Published anchor: lat={lat}, lon={lon} from {self._transform_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to read transform file {self._transform_file}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AnchorPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
