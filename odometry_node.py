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

        # --- CONFIGURATION (YOU MUST CHANGE THIS) ---
        self.wheel_diameter = 0.068
        self.wheel_base = 0.30
        self.ticks_per_rev = 4600
        
        # --- POLARITY CORRECTION (FIX FOR REVERSED DIRECTION) ---
        # Jika maju hasilnya negatif, ubah 1 menjadi -1
        self.polarity_left = -1  
        self.polarity_right = -1 
        
        # --- CALCULATED CONSTANTS ---
        self.m_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # --- ROBOT STATE ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.initialized = False

        # --- ROS COMMUNICATION ---
        self.sub_enc = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            10
        )
        
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Odometry Node Started. Waiting for wheel movement...')

    def encoder_callback(self, msg):
        # Ambil data raw dan terapkan polaritas
        current_left = msg.data[0] * self.polarity_left
        current_right = msg.data[1] * self.polarity_right

        # Handle first run
        if not self.initialized:
            self.prev_left_ticks = current_left
            self.prev_right_ticks = current_right
            self.initialized = True
            return

        # 1. Calculate change in ticks
        d_left_ticks = current_left - self.prev_left_ticks
        d_right_ticks = current_right - self.prev_right_ticks

        # --- DEBUG PRINT (
