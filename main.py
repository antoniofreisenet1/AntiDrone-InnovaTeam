import math
from time import sleep, time

from ev3dev2.motor import MediumMotor
from ev3dev2.sound import Sound
from smbus2 import SMBus
from ev3dev2.sensor import LegoPort

motor = MediumMotor("outA")

velocidad_base = 30
factor_correccion = 2

# Signatures we'll be accepting (SIGNATURE 2 FOR GREEN)
sigs = 2

# I2C bus and address
bus = SMBus(3)
address = 0x54

def motor_test():
    try:
        motor.run_direct(duty_cycle_sp=velocidad_base)
        sleep(1)
        motor.stop()
    except KeyboardInterrupt:
        print("Stopped by user")

def calibrateAll():
    # Pixy2 has a resolution of 328 * 200
    # Make sure that the function will properly move the camera/shooter
    target_x = 328 / 2
    target_y = 200 / 2

    current_x, current_y = com_queue.get()

    # Calculate the difference between the current position and the target position
    dx = target_x - current_x
    dy = target_y - current_y

    # Calculate the angle (in radians) between the current position vector and the target position vector
    angle = math.atan2(dy, dx)

    # Convert the angle to degrees and add 60 degrees to create a vector at a 60-degree angle
    angle_degrees = math.degrees(angle) + 60

    # Calculate the length of the vector based on the speed_sp
    speed_sp = 200
    vector_length = math.sqrt(dx**2 + dy**2)

    # Calculate proportional control adjustments
    kp = 0.1  # Adjust this value based on experimentation
    proportional_adjustment_x = kp * dx
    proportional_adjustment_y = kp * dy

    # Calculate the new target position using the vector at a 60-degree angle and proportional adjustments
    new_target_x = current_x + vector_length * math.cos(math.radians(angle_degrees)) + proportional_adjustment_x
    new_target_y = current_y + vector_length * math.sin(math.radians(angle_degrees)) + proportional_adjustment_y

    # Move the motors to the new target position
    motor.run_to_abs_pos(position_sp=int(new_target_x), speed_sp=speed_sp)
    motor.run_to_abs_pos(position_sp=int(new_target_y), speed_sp=speed_sp)

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

    # Formula for measuring the distance to an object (v2):
    # distance = (AVERAGE_GREEN_OBJECT_SIZE)
    # / (2 * green_object_size * math.tan(math.radians(60 / 2)))

    AVERAGE_GREEN_OBJECT_SIZE = 1000  # size in pixels
    green_object_size = w
    distance = AVERAGE_GREEN_OBJECT_SIZE / (2 * green_object_size * math.tan(math.radians(60 / 2)))

    # Calibrate the motors to center the target in the camera
    calibrateAll()

    return distance

try:
    motor_test()
    cam_setup()
    sound = Sound()
    while True:
        dist = cam_detect()
        if 150 > dist > 1:
            sound.speak('Detected')
            print("Object detected at: ", dist)
        velocidad_motor = max(-100, min(100, velocidad_base))
        motor.run_direct(duty_cycle_sp=velocidad_motor)
        sleep(0.1)
        motor.stop()

except KeyboardInterrupt:
    print("Stop!")
    motor.stop()
