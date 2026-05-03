import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aleks/Joe-thesis-2026-Sensor-integration/semubot_ros2_collision_avoidance_ws/install/ultrasonic_nav'
