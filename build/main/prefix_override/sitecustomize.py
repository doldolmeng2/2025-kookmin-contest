import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/doldolmeng2/xycar_ws/src/orda/install/main'
