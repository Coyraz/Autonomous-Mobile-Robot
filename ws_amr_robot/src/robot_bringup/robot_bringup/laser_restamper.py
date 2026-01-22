#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserRestamper(Node):
    def __init__(self):
        super().__init__('laser_restamper')
        
        # 1. Listen to the raw data from the Lidar Driver
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # 2. Prepare to publish the fixed data
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_restamped',
            10
        )
        
        self.get_logger().info('Laser Restamper Active: Synchronizing timestamps...')
    
    def scan_callback(self, msg):
        # Create a new empty message
        restamped_msg = LaserScan()
        
        # Copy ALL data fields from the original message
        # We want the distance data to stay exactly the same
        restamped_msg.header = msg.header
        restamped_msg.angle_min = msg.angle_min
        restamped_msg.angle_max = msg.angle_max
        restamped_msg.angle_increment = msg.angle_increment
        restamped_msg.time_increment = msg.time_increment
        restamped_msg.scan_time = msg.scan_time
        restamped_msg.range_min = msg.range_min
        restamped_msg.range_max = msg.range_max
        restamped_msg.ranges = msg.ranges
        restamped_msg.intensities = msg.intensities
        
        # THE FIX: Overwrite the timestamp with the current system time
        # This makes SLAM Toolbox think the data arrived "just now"
        restamped_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Send the fixed message
        self.scan_pub.publish(restamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
