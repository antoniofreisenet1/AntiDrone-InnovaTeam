from time import *

import ev3dev2._platform.ev3
from smbus2 import SMBus
from ev3dev2.auto import *
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *

motor = MediumMotor(OUTPUT_A)


velocidad_base = 30
factor_correccion = 2

# Signatures we'll be accepting (SIGNATURE 2 FOR GREEN)
sigs = 2

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
    bus.write_i2c_block_data(0x54, 0, data)
    block = bus.read_i2c_block_data(0x54, 0, 13)
    print("Firmware version: {}.{}\n".format(str(block[8]), str(block[9])))



def cam_detect():

    data = [174, 193, 32, 2, sigs, 1]

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

    # Formula for measuring the distance to an object (v1):
    # distance(final)= distance(initial)times(x) the square root of {initial area divided by measured area}

    # Formula for measuring the distance to an object (v2):
    # distance = (AVERAGE_GREEN_OBJECT_SIZE)
    # / (2 * green_object_size * math.tan(math.radians(39.6 / 2)))

    AVERAGE_GREEN_OBJECT_SIZE = 10  # size in cm
    green_object_size = w
    distance = AVERAGE_GREEN_OBJECT_SIZE / (2*green_object_size * math.tan(math.radians(60/2)))

    return distance

try:

    motor_test()
    cam_setup()
    sound = Sound()
    while True:
        dist = cam_detect()
        if dist < 150:
            sound.speak('Detected')
            print("Object detected at: ", dist)
        velocidad_motor = max(-100, min(100, velocidad_base))
        motor.run_direct(duty_cycle_sp=velocidad_motor)
        time.sleep(0.1)
        motor.stop()

except KeyboardInterrupt:
    print("Stop!")
    motor.stop()
