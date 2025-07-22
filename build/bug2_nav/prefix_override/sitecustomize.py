import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/phumint/turtlebot3_ws/src/bug2_nav/install/bug2_nav'
