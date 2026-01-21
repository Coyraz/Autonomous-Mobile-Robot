#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray 
import serial
import json
import time

class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_uart_bridge')
        
        # --- CONFIGURATION ---
        self.port_name = '/dev/ttyUSB0' # Adjust if needed
        self.baud_rate = 115200
        
        # --- ENCODER STATE TRACKING ---
        # We need to track the previous raw values to calculate distance
        self.prev_left_raw = 0
        self.prev_right_raw = 0
        
        # These accumulate the TOTAL ticks since start (can go to millions)
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        
        self.is_first_message = True

        # --- SERIAL CONNECTION ---
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected to STM32 at {self.port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            exit(1)

        # --- ROS PUBLISHER ---
        # We publish the TOTAL ACCUMULATED ticks
        self.pub_encoders = self.create_publisher(Int32MultiArray, 'wheel_encoders', 10)
        
        # Read loop at 50Hz (0.02s)
        self.timer = self.create_timer(0.02, self.read_serial_data)

    def calculate_delta(self, current, previous):
        """
        Calculates the difference between two 16-bit signed integers.
        Handles the overflow/wrap-around logic.
        Range of input: -32768 to 32767
        """
        delta = current - previous
        
        # Handle Wrap-Around (Overflow)
        # If delta is too large positive, it means we wrapped backward
        if delta > 32768:
            delta -= 65536
        # If delta is too large negative, it means we wrapped forward
        elif delta < -32768:
            delta += 65536
            
        return delta

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Parse JSON: {"l": 123, "r": -456}
                if line.startswith('{') and line.endswith('}'):
                    data = json.loads(line)
                    
                    raw_left = data.get('l', 0)
                    raw_right = data.get('r', 0)

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

                    # Save current raw values for next time
                    self.prev_left_raw = raw_left
                    self.prev_right_raw = raw_right

                    # Publish to ROS
                    msg = Int32MultiArray()
                    msg.data = [self.total_left_ticks, self.total_right_ticks]
                    self.pub_encoders.publish(msg)

            except (json.JSONDecodeError, UnicodeDecodeError):
                pass # Ignore bad packets
            except Exception as e:
                self.get_logger().warn(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
