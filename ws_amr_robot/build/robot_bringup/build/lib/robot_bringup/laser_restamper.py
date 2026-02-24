#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data # Hanya digunakan untuk mendengarkan Lidar

class LaserRestamper(Node):
    def __init__(self):
        super().__init__('laser_restamper')
        
        # 1. SUBSCRIBER: Mendengarkan Lidar asli.
        # WAJIB menggunakan qos_profile_sensor_data (Best Effort) karena sllidar memancarkan Best Effort.
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data 
        )
        
        # 2. PUBLISHER: Memancarkan data yang sudah distempel ulang.
        # WAJIB menggunakan angka 10 (Reliable) karena SLAM Toolbox dan RViz 
        # secara bawaan menuntut koneksi yang Reliable.
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_restamped',
            10 
        )
        
        self.get_logger().info('Laser Restamper Active: Translating QoS and Fixing Time...')
    
    def scan_callback(self, msg):
        restamped_msg = LaserScan()
        
        # Copy data pengukuran (jarak)
        restamped_msg.angle_min = msg.angle_min
        restamped_msg.angle_max = msg.angle_max
        restamped_msg.angle_increment = msg.angle_increment
        restamped_msg.time_increment = msg.time_increment
        restamped_msg.scan_time = msg.scan_time
        restamped_msg.range_min = msg.range_min
        restamped_msg.range_max = msg.range_max
        restamped_msg.ranges = msg.ranges
        restamped_msg.intensities = msg.intensities
        
        # 1. Paksa nama frame agar SESUAI dengan Static TF di launch file
        restamped_msg.header.frame_id = 'laser_frame' 
        
        # 2. Paksa waktu menjadi Waktu Sekarang (System Time)
        restamped_msg.header.stamp = self.get_clock().now().to_msg()
        # ----------------
        
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
