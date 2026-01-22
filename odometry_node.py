#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- KONFIGURASI FISIK ---
        self.wheel_diameter = 0.068
        self.wheel_base = 0.30
        self.ticks_per_rev = 4600.0
        
        # --- POLARITY ---
        self.polarity_left = -1.0  
        self.polarity_right = -1.0 
        
        # --- CONSTANTS ---
        self.m_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # --- STATE ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.initialized = False

        # --- COMM ---
        self.sub_enc = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            10
        )
        
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Odometry Node Started.')

    def encoder_callback(self, msg):
        current_left = msg.data[0] * self.polarity_left
        current_right = msg.data[1] * self.polarity_right

        if not self.initialized:
            self.prev_left_ticks = current_left
            self.prev_right_ticks = current_right
            self.initialized = True
            return

        d_left_ticks = current_left - self.prev_left_ticks
        d_right_ticks = current_right - self.prev_right_ticks

        if d_left_ticks != 0 or d_right_ticks != 0:
             self.get_logger().info(f'Gerak: dL={d_left_ticks} dR={d_right_ticks} | Posisi X={self.x:.3f}')

        self.prev_left_ticks = current_left
        self.prev_right_ticks = current_right

        d_left_meters = d_left_ticks * self.m_per_tick
        d_right_meters = d_right_ticks * self.m_per_tick

        d_center = (d_left_meters + d_right_meters) / 2.0
        d_theta = (d_right_meters - d_left_meters) / self.wheel_base

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta
        
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odometry()

    def publish_odometry(self):
        # Ambil waktu sekarang
        now = self.get_clock().now()
        current_time = now.to_msg()
        
        q = self.euler_to_quaternion(0, 0, self.theta)

        # 1. Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        
        # PENTING: Kirim TF dulu sebelum Odom message
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Topic (/odom)
        odom = Odometry()
        odom.header.stamp = current_time # Pastikan timestamp sama persis dengan TF
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        
        self.pub_odom.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
