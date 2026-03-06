#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu  # [TAMBAHAN MUTLAK] Import standar pesan inersia ROS 2

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        
        # Serial connection
        self.port_name = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.ser.reset_input_buffer()
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
        
        # --- PUBLISHERS ---
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10) # [TAMBAHAN] Publisher data inersia
        
        # --- SUBSCRIBERS ---
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Current velocity command
        self.current_v = 0  # mm/s
        self.current_w = 0  # mrad/s
        
        # [PERBAIKAN KRITIS] Dipercepat ke 20Hz untuk mencegah penumpukan bufer (bottle-neck)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info('STM32 Bridge Active (Read Encoders+IMU, Write Cmd_Vel)')
    
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
        """Combined write command + read encoder + IMU data"""
        
        # 1. Send command to STM32
        try:
            cmd_str = f"V:{self.current_v},W:{self.current_w}\r\n"
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            return
        
        # 2. Read data from STM32
        try:
            latest_line = None
            
            # [PERBAIKAN LOGIKA] Menguras seluruh antrean bufer serial.
            # Mengeliminasi latensi dengan hanya menyimpan string yang paling baru diterima.
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    latest_line = line
                    
            if latest_line is not None:
                try:
                    data = json.loads(latest_line)
                    
                    # --- A. OLAH DATA ENCODER ---
                    if 'l' in data and 'r' in data:
                        raw_left = data['l']
                        raw_right = data['r']
                        
                        if self.is_first_message:
                            self.prev_left_raw = raw_left
                            self.prev_right_raw = raw_right
                            self.is_first_message = False
                            return
                        
                        delta_left = self.calculate_delta(raw_left, self.prev_left_raw)
                        delta_right = self.calculate_delta(raw_right, self.prev_right_raw)
                        
                        self.total_left_ticks += delta_left
                        self.total_right_ticks += delta_right
                        
                        self.prev_left_raw = raw_left
                        self.prev_right_raw = raw_right
                        
                        msg_enc = Int32MultiArray()
                        msg_enc.data = [self.total_left_ticks, self.total_right_ticks]
                        self.encoder_pub.publish(msg_enc)
                        
                    # --- B. OLAH DATA IMU ---
                    # Menangkap variabel giroskop dan akselerometer jika ada di dalam JSON
                    if 'gz' in data:
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'base_link' # Mengikat orientasi ini ke badan robot
                        
                        # Transformasi Satuan Fisika Dasar
                        # Gyro Z: mili-radian per detik -> radian per detik
                        imu_msg.angular_velocity.z = float(data['gz']) / 1000.0
                        
                        # Accel: cm per detik kuadrat -> meter per detik kuadrat
                        imu_msg.linear_acceleration.x = float(data.get('ax', 0)) / 100.0
                        imu_msg.linear_acceleration.y = float(data.get('ay', 0)) / 100.0
                        imu_msg.linear_acceleration.z = float(data.get('az', 980)) / 100.0
                        
                        # Deklarasi Matriks Kovariansi (Kepercayaan Data)
                        # Dibutuhkan mutlak agar EKF tidak menolak data ini
                        imu_msg.angular_velocity_covariance[8] = 0.005 
                        imu_msg.linear_acceleration_covariance[0] = 0.05
                        imu_msg.linear_acceleration_covariance[4] = 0.05
                        imu_msg.linear_acceleration_covariance[8] = 0.05
                        
                        # Mematikan validasi orientasi absolut (karena MPU6050 tidak punya kompas)
                        imu_msg.orientation_covariance[0] = -1.0 
                        
                        self.imu_pub.publish(imu_msg)
                        
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
