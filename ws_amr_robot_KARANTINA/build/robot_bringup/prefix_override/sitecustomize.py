import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kanal-raspi5/Autonomous-Mobile-Robot/ws_amr_robot/install/robot_bringup'
