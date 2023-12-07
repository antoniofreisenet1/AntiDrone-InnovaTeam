from time import *

import ev3dev2._platform.ev3
from ev3dev2.port import LegoPort
from pixycamev3 import *
from smbus2 import SMBus
from ev3dev.auto import *
from ev3dev2.motor import *
from ev3dev2.sensor import *

motor = MediumMotor(OUTPUT_A)


velocidad_base = 30
factor_correccion = 2

# Signatures we'll be accepting
sigs = 'SIG2'

# I2C bus and address
bus = SMBus(3)
address = 0x54

def motor_test():
    # COMPONENT CONNECTIVITY TEST 1 - MOTOR TEST (CAMERA MOTOR)


    try:
        motor.run_direct(duty_cycle_sp=velocidad_base)
        sleep(1)
        motor.stop()
    except KeyboardInterrupt:
        # TODO: SPECIFY EXCEPTION TYPES (OPTIONAL)
        print("Stopped by user")


def cam_setup():
    # COMPONENT CONNECTIVITY TEST 2 - CAMERA TEST
    # set LEGO port for Pixy on Input 1
    in1 = LegoPort(ev3dev2._platform.ev3.INPUT_1)
    in1.mode = 'other-i2c'

    # wait for the port to get ready
    sleep(2)

    # request firmware version (just to test)

    data = [174, 193, 14, 0]
    bus.write_i2c_block_data(address, 0, data)
    block = bus.read_i2c_block_data(address, 0, 13)
    print("Firmware version: {}.{}\n".format(str(block[8]), str(block[9])))



def cam_detect():

    data = [174, 193, 32, 2, sigs, 1]  # this '1' means it will only detect the largest object

    # Request block
    bus.write_i2c_block_data(address, 0, data)

    # Read block request
    block = bus.read_i2c_block_data(address, 0, 20)

    # Extract positional information
    x = block[9] * 256 + block[8]
    y = block[11] * 256 + block[10]
    w = block[13] * 256 + block[12]
    h = block[15] * 256 + block[14]

    # DOCS: https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:porting_guide#pixy2-serial-protocol-packet-reference
    # TODO: implement getBlocks function for data transfer (see docs above)
try:
    while True:

        velocidad_motor = max(-100, min(100, velocidad_base))
        motor.run_direct(duty_cycle_sp=velocidad_motor)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stop!")
    motor.stop()