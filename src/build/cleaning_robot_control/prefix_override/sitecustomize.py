import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yys/cleaning_robot_ws/src/install/cleaning_robot_control'
