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
        
        # --- KONFIGURASI SERIAL ---
        # Pastikan port ini sesuai dengan hasil 'ls /dev/ttyUSB*'
        self.port_name = '/dev/ttyUSB0'
        self.baud_rate = 115200
        
        # Variabel untuk menyimpan data terakhir (untuk safety)
        self.left_tick = 0
        self.right_tick = 0

        # Mencoba membuka koneksi serial
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Berhasil terhubung ke STM32 di {self.port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal membuka serial port: {e}')
            # Kita matikan node jika serial gagal, karena percuma jalan tanpa data
            exit(1)

        # --- KONFIGURASI ROS 2 ---
        # Kita gunakan Int32MultiArray: [encoder_kiri, encoder_kanan]
        self.publisher_ = self.create_publisher(Int32MultiArray, 'wheel_encoders', 10)
        
        # Timer: Membaca data secepat mungkin (0.01 detik = 100Hz)
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        # Cek apakah ada data yang masuk di kabel
        if self.ser.in_waiting > 0:
            try:
                # 1. Baca satu baris teks (decode dari bytes ke string)
                line = self.ser.readline().decode('utf-8').strip()
                
                # 2. Validasi format JSON sederhana
                # Kita cek apakah diawali '{' dan diakhiri '}' agar tidak parsing sampah
                if line.startswith('{') and line.endswith('}'):
                    
                    # 3. Parsing JSON ke Dictionary Python
                    data = json.loads(line)
                    
                    # Ambil data 'l' dan 'r', gunakan nilai lama jika key tidak ada
                    self.left_tick = data.get('l', self.left_tick)
                    self.right_tick = data.get('r', self.right_tick)

                    # 4. Bungkus data ke pesan ROS
                    msg = Int32MultiArray()
                    msg.data = [self.left_tick, self.right_tick]
                    
                    # 5. Kirim (Publish)
                    self.publisher_.publish(msg)
                    
                    # Debugging (Opsional: Hidupkan jika ingin melihat log di terminal)
                    # self.get_logger().info(f'Publish: L={self.left_tick}, R={self.right_tick}')

            except json.JSONDecodeError:
                # Ini wajar terjadi sesekali jika kabel goyang atau data terpotong
                self.get_logger().warn(f'Data Rusak (JSON Error): {line}')
            except UnicodeDecodeError:
                self.get_logger().warn('Data Rusak (Encoding Error)')
            except Exception as e:
                self.get_logger().error(f'Error Tak Terduga: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Stm32Bridge()
    
    try:
        # Biarkan node hidup terus
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Menangani Ctrl+C dengan rapi
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
