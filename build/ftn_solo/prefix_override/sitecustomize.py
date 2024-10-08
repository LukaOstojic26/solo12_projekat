import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/luka/HDD/solo12/src/ftn_solo/install/ftn_solo'
