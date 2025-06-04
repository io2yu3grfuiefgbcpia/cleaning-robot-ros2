import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yys/cleaning_robot_ws/install/cleaning_robot_navigation'
