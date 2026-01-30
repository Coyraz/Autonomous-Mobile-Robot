#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publisher for velocity commands (ROS standard topic)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Velocity settings (m/s and rad/s)
        self.linear_speed = 0.6    # 0.6 m/s = 600 mm/s
        self.angular_speed = 0.7   # 0.7 rad/s = 700 mrad/s
        
        self.current_v = 0.0
        self.current_w = 0.0
        self.running = True
        
        # Timer to publish commands at 20Hz
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)
        
        self.get_logger().info('Teleop Ready! (ROS Topic Mode)')
        self.print_usage()
    
    def print_usage(self):
        print("\n" + "="*60)
        print(" "*15 + "🤖 ROBOT TELEOP CONTROL 🤖")
        print("="*60)
        print("\n  Controls:")
        print("    ↑ UP     : Forward")
        print("    ↓ DOWN   : Backward")
        print("    ← LEFT   : Turn Left")
        print("    → RIGHT  : Turn Right")
        print("    SPACE    : Stop")
        print("    Q        : Quit")
        print("\n" + "="*60)
        print("  Status: Waiting for input...\n")
    
    def publish_cmd_vel(self):
        """Publish Twist message at 20Hz"""
        if not self.running:
            return
        
        msg = Twist()
        msg.linear.x = self.current_v
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.current_w
        
        self.cmd_vel_pub.publish(msg)
    
    def update_display(self, status_msg):
        """Update status display"""
        print(f"\r  Status: {status_msg:<50}", end='', flush=True)
    
    def keyboard_loop(self):
        """Keyboard input thread"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                key = sys.stdin.read(1)
                
                if key == '\x1b':  # ESC sequence
                    key += sys.stdin.read(2)
                    
                    if key == '\x1b[A':  # UP
                        self.current_v = self.linear_speed
                        self.current_w = 0.0
                        self.update_display(f"⬆️  FORWARD (V={self.current_v:.1f} m/s)")
                        
                    elif key == '\x1b[B':  # DOWN
                        self.current_v = -self.linear_speed
                        self.current_w = 0.0
                        self.update_display(f"⬇️  BACKWARD (V={self.current_v:.1f} m/s)")
                        
                    elif key == '\x1b[D':  # LEFT
                        self.current_v = 0.0
                        self.current_w = self.angular_speed
                        self.update_display(f"⬅️  TURN LEFT (W={self.current_w:.1f} rad/s)")
                        
                    elif key == '\x1b[C':  # RIGHT
                        self.current_v = 0.0
                        self.current_w = -self.angular_speed
                        self.update_display(f"➡️  TURN RIGHT (W={self.current_w:.1f} rad/s)")
                
                elif key == ' ':
                    self.current_v = 0.0
                    self.current_w = 0.0
                    self.update_display("⏹️  STOPPED")
                
                elif key == 'q' or key == 'Q':
                    self.current_v = 0.0
                    self.current_w = 0.0
                    print("\n\n  Shutting down teleop...")
                    self.running = False
                    break
        
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    # Start keyboard thread
    keyboard_thread = threading.Thread(target=node.keyboard_loop, daemon=False)
    keyboard_thread.start()
    
    # Spin ROS
    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n\n  Interrupted by user")
    finally:
        node.running = False
        keyboard_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
        print("  Teleop shutdown complete.\n")

if __name__ == '__main__':
    main()
