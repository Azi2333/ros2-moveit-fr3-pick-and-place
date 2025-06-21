import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/azi888/rf_ws/src/lab4_pkg/install/lab4_pkg'
