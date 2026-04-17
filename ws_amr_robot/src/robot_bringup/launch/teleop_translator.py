#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TeleopTranslator(Node):
    def __init__(self):
        super().__init__('teleop_translator')
        
        # 1. MENDENGARKAN FOXGLOVE (Menerima Twist murni yang bodoh)
        self.subscription = self.create_subscription(
            Twist,
            '/foxglove_teleop_raw', # Topik baru khusus untuk Foxglove
            self.listener_callback,
            10
        )
        
        # 2. BERBICARA KE TWIST_MUX (Meneruskan TwistStamped yang pintar)
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/cmd_vel_teleop',      # Topik yang didengarkan oleh twist_mux
            10
        )
        
        self.get_logger().info('Teleop Translator Active: Bridging Foxglove (Twist) -> Twist_Mux (TwistStamped)')

    def listener_callback(self, msg_in):
        # Membungkus data Twist mentah ke dalam amplop TwistStamped
        msg_out = TwistStamped()
        
        # Injeksi stempel waktu real-time dari Raspberry Pi
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'base_link'
        
        # Salin data kecepatan
        msg_out.twist.linear.x = msg_in.linear.x
        msg_out.twist.angular.z = msg_in.angular.z
        
        # Publikasikan ke sistem
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
