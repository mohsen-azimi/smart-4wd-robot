from utils import UGV# , UAV
from pynput import keyboard

ugv = UGV(port='COM13', baudrate=9600, timeout=0.1, wheel_speed=30)
# uav.drone_conect().drone_takeoff()

# ugv.test(wheel=0)
ugv.control_by_keyboard()








