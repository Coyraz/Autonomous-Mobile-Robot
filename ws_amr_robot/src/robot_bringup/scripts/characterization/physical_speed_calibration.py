#!/usr/bin/env python3
"""
physical_speed_calibration.py
-----------------------------
Calibrate the robot's REAL speed with a stopwatch + tape measure / protractor,
not the sensors. Encoder, IMU, and odometry can all be wrong; a tape measure
cannot. The script only controls the timing precisely. YOU do the measuring.

It offers TWO tests, you choose when you run it:

  1) LINEAR  : drives forward at a fixed speed for a fixed time, then stops.
               You measure the distance traveled.
               real speed = distance / duration   (m/s)

  2) ANGULAR : rotates in place at a fixed rate for a fixed time, then stops.
               You count full turns + leftover angle.
               real speed = (turns*2*pi + extra_rad) / duration   (rad/s)

USE EACH TEST FOR:
  - MAX speed : raise the value until distance/turns stop increasing.
  - DEADBAND  : lower the value until the robot no longer moves at all.
    (deadband = lowest command that still produces motion)

HOW TO USE:
  Terminal 1: ~/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py
  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    python3 .../characterization/physical_speed_calibration.py
  Then follow the menu.

SAFETY:
  - LINEAR needs a clear aisle longer than speed*duration. Hand on the e-stop.
  - ANGULAR stays in place; keep the area around the robot clear.
"""

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SpeedCalib(Node):
    def __init__(self):
        super().__init__('physical_speed_calibration')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, mode, value):
        msg = Twist()
        if mode == 'linear':
            msg.linear.x = value
        else:
            msg.angular.z = value
        self.pub.publish(msg)

    def stop(self):
        for _ in range(8):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_for(self, mode, value, duration):
        """Hold the command for exactly `duration` seconds, then stop."""
        start = time.time()
        last = -1
        while True:
            elapsed = time.time() - start
            if elapsed >= duration:
                break
            self.send(mode, value)
            rclpy.spin_once(self, timeout_sec=0.05)
            secs_left = int(duration - elapsed)
            if secs_left != last:
                print(f'  {secs_left:3d}s remaining...')
                last = secs_left
        self.stop()


def get_float(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print('  Please enter a number.')


def linear_test(node):
    value = get_float('  Forward speed to command (m/s, e.g. 0.15): ')
    duration = get_float('  Duration (seconds, e.g. 10): ')
    reach = value * duration
    print()
    print(f'  Robot drives forward ~{reach:.2f} m at {value} m/s for {duration:.0f}s.')
    print('  Tape-mark the FRONT of the robot NOW (start). Need a clear aisle.')
    print('  Hand on the emergency stop.')
    input('  Press ENTER to start...')
    print()
    node.run_for('linear', value, duration)
    print()
    print('  STOPPED. Measure the distance between the two tape marks (meters).')
    print(f'    real_speed = distance / {duration:.0f}')
    print(f'    example: 1.50 m  ->  {1.50/duration:.3f} m/s')
    print(f'  You commanded {value} m/s. If measured is much lower, the motor/')
    print('  PID cannot reach it (or you are below the deadband).')


def angular_test(node):
    value = get_float('  Rotation rate to command (rad/s, e.g. 0.60): ')
    duration = get_float('  Duration (seconds, e.g. 30): ')
    turns = value * duration / (2 * math.pi)
    print()
    print(f'  Robot rotates in place ~{turns:.1f} full turns (estimate) at '
          f'{value} rad/s for {duration:.0f}s.')
    print('  Tape an arrow on top pointing forward, mark its start heading.')
    print('  Keep the area clear. Hand on the emergency stop.')
    input('  Press ENTER to start...')
    print()
    node.run_for('angular', value, duration)
    print()
    print('  STOPPED. Count full turns + measure leftover angle (protractor).')
    print(f'    real_speed = (turns*2*pi + extra_rad) / {duration:.0f}')
    print(f'    example: 3 turns + 30deg  ->  '
          f'{(3*2*math.pi + math.radians(30))/duration:.3f} rad/s')
    print(f'  You commanded {value} rad/s. If measured is much lower, the motor/')
    print('  PID cannot reach it (or you are below the rotation deadband).')


def main():
    rclpy.init()
    node = SpeedCalib()

    print()
    print('=' * 60)
    print('  PHYSICAL SPEED CALIBRATION (stopwatch + tape/protractor)')
    print('=' * 60)
    print('  Choose a test:')
    print('    1) LINEAR  - measure forward speed with a tape measure')
    print('    2) ANGULAR - measure rotation speed with a protractor')
    print()

    choice = input('  Enter 1 or 2: ').strip()
    print()
    if choice == '1':
        linear_test(node)
    elif choice == '2':
        angular_test(node)
    else:
        print('  Invalid choice. Run again and enter 1 or 2.')

    print()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
