import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/helloosy/250805/2025-kookmin-contest/install/manual_drive'
