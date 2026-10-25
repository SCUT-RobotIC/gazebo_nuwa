import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wyq/work/nuwa_sim/robots/install/mini_car_control'
