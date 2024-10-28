import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ngoc/Workspace/Graduation_Project/install/teleop_twist_keyboard'
