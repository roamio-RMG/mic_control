import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roamio42/Desktop/mic_control/src/install/keyboard_control'
