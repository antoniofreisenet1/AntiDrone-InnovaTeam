from time import *
from queue import Queue
import threading

import ev3dev2._platform.ev3
from smbus2 import SMBus
from ev3dev2.auto import *
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *

def motor_test():
    # Test both motors
    try:
        horizontal_motor.run_to_abs_pos(position_sp=0, speed_sp=200)
        vertical_motor.run_to_abs_pos(position_sp=0, speed_sp=200)
        horizontal_motor.wait_while('running')
        vertical_motor.wait_while('running')
    except KeyboardInterrupt:
        print("Stopped by user")