#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

class LaserRestamper(Node):
    def __init__(self):
        super().__init__('laser_restamper')
        
        # 1. SUBSCRIBER: Mendengarkan Lidar asli (Best Effort)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data 
        )
        
        # 2. PUBLISHER: Memancarkan data yang sudah distempel ulang (Reliable)
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_restamped',
            10 
        )
        
        self.get_logger().info('Laser Restamper Active: BACKDATING time by 500ms for Slow EKF Sync...')
    
    def scan_callback(self, msg):
        restamped_msg = LaserScan()
        
        # Menyalin murni data fisik laser
        restamped_msg.angle_min = msg.angle_min
        restamped_msg.angle_max = msg.angle_max
        restamped_msg.angle_increment = msg.angle_increment
        restamped_msg.time_increment = msg.time_increment
        restamped_msg.scan_time = msg.scan_time
        restamped_msg.range_min = msg.range_min
        restamped_msg.range_max = msg.range_max
        restamped_msg.ranges = msg.ranges
        restamped_msg.intensities = msg.intensities
        
        # 1. Menyelaraskan nama bingkai spasial
        restamped_msg.header.frame_id = 'laser_frame' 
        
        # --- [OPERASI BEDAH MUTLAK: EKSTREMASI TIME BACKDATING] ---
        # Mengompensasi CPU Raspberry Pi yang tersedak. EKF butuh >300ms untuk berpikir.
        # Kita memundurkan waktu Lidar sebanyak 500 milidetik (0.5 detik) ke masa lalu.
        now = self.get_clock().now()
        offset = Duration(seconds=0, nanoseconds=500000000) # 500 ms
        past_time = now - offset
        
        restamped_msg.header.stamp = past_time.to_msg()
        # -----------------------------------------------
        
        self.scan_pub.publish(restamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
