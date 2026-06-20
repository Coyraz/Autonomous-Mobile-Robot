#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        
        # Serial connection
        self.port_name = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
        self.baud_rate = 115200
        
        try:
            self.get_logger().info('Waiting for STM32 to stabilize...')
            time.sleep(2.0)  # Wait for STM32 to be running
            
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.ser.reset_input_buffer()  # Clear any old data
            self.ser.reset_output_buffer()
            
            self.get_logger().info(f'Connected to STM32 at {self.port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            exit(1)
        
        # Encoder tracking (overflow handling)
        self.prev_left_raw = 0
        self.prev_right_raw = 0
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        self.is_first_message = True
        
        # Publishers
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)  # ← ADDED
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Current velocity command
        self.current_v = 0  # mm/s
        self.current_w = 0  # mrad/s
        
        # Timer at 10Hz (slower = more stable)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('STM32 Bridge Active (Encoders + IMU)')  # ← UPDATED
    
    def calculate_delta(self, current, previous):
        """Calculate delta with 16-bit overflow handling"""
        delta = current - previous
        if delta > 32768:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to STM32 format"""
        self.current_v = int(msg.linear.x * 1000)   # m/s to mm/s
        self.current_w = int(msg.angular.z * 1000)  # rad/s to mrad/s
        
        # Clamp values
        self.current_v = max(-1000, min(1000, self.current_v))
        self.current_w = max(-2000, min(2000, self.current_w))
    
    def timer_callback(self):
        """Combined read encoder + IMU data + write command at 10Hz"""
        
        # 1. Send command to STM32 (Tetap sama)
        try:
            cmd_str = f"V:{self.current_v},W:{self.current_w}\r\n"
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            return
        
        # 2. Read data from STM32 (PERBAIKAN LOGIKA KEMACETAN)
        try:
            latest_line = None
            
            # Kuras seluruh isi buffer sampai kosong, hanya simpan baris terakhir yang paling baru
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    latest_line = line
                    
            # Jika ada baris yang valid setelah buffer dikuras, baru lakukan parsing JSON
            if latest_line is not None:
                try:
                    data = json.loads(latest_line)
                    
                    # Parse encoder data (always present)
                    if 'l' in data and 'r' in data:
                        raw_left = data['l']
                        raw_right = data['r']
                        
                        # Initialize on first run
                        if self.is_first_message:
                            self.prev_left_raw = raw_left
                            self.prev_right_raw = raw_right
                            self.is_first_message = False
                            return
                        
                        # Calculate change since last read
                        delta_left = self.calculate_delta(raw_left, self.prev_left_raw)
                        delta_right = self.calculate_delta(raw_right, self.prev_right_raw)
                        
                        # Update totals
                        self.total_left_ticks += delta_left
                        self.total_right_ticks += delta_right
                        
                        # Save current raw values
                        self.prev_left_raw = raw_left
                        self.prev_right_raw = raw_right
                        
                        # Publish encoder data
                        encoder_msg = Int32MultiArray()
                        encoder_msg.data = [self.total_left_ticks, self.total_right_ticks]
                        self.encoder_pub.publish(encoder_msg)
                        
                        # ... (Bagian IMU Parsing tetap biarkan seperti aslinya jika Anda mau, 
                        # meskipun saat ini fisiknya dicabut)
                        
                except json.JSONDecodeError:
                    pass
                        
        except Exception as e:
            self.get_logger().error(f'Read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
