import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/outthawazoo/traxxas_ws/install/traxxas_slam'
