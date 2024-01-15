from time import *
from queue import Queue
import threading

import ev3dev2._platform.ev3
from smbus2 import SMBus
from ev3dev2.auto import *
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *

sound = Sound()

velocidad_base = 30
factor_correccion = 2

# Queue definition
com_queue = Queue()

# Signatures we'll be accepting (SIGNATURE 2 FOR GREEN)
sigs = 1

# I2C bus and address
bus = SMBus(3)
address = 0x54
# Define motors

# Define scan range (in degrees)
horizontal_range = [0, 360]  # Range for horizontal scanning
vertical_range = [-90, 0]  # Range for vertical scanning


def motor_test():
    # Test both motors
    try:
        horizontal_motor.run_to_abs_pos(position_sp=0, speed_sp=200)
        vertical_motor.run_to_abs_pos(position_sp=0, speed_sp=200)
        horizontal_motor.wait_while('running')
        vertical_motor.wait_while('running')
    except KeyboardInterrupt:
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
    print(block)
    # Extract positional information
    x = block[9] * 256 + block[8]
    y = block[11] * 256 + block[10]
    w = block[13] * 256 + block[12]
    h = block[15] * 256 + block[14]
    print(x,y,w,h)
    # DOCS: https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:porting_guide#pixy2-serial-protocol-packet-reference

    # Formula for measuring the distance to an object (v1):
    # distance(final)= distance(initial)times(x) the square root of {initial area divided by measured area}

    # Formula for measuring the distance to an object (v2):
    # distance = (AVERAGE_GREEN_OBJECT_SIZE)
    # / (2 * green_object_size * math.tan(math.radians(39.6 / 2)))
    # TODO: Remake the formula to better reflect the distance to an object.

    AVERAGE_GREEN_OBJECT_SIZE = 1000  # size in pixels
    green_object_size = w
    distance = AVERAGE_GREEN_OBJECT_SIZE / (2 * green_object_size * math.tan(math.radians(60 / 2)))

    return distance


def scan_and_detect():
    # Scanning loop
    h_pos = horizontal_range[0]
    for v_pos in range(vertical_range[0], vertical_range[1] + 1, 10):  # Vertical scan
        vertical_motor.run_to_abs_pos(position_sp=v_pos, speed_sp=200)
        while h_pos < horizontal_range[1]:  # Horizontal scan
            dist = cam_detect()
            if 150 > dist > 1:
                sound.speak('Destruction')
                print("Object detected at:", dist, "Horizontal:", h_pos, "Vertical: ", v_pos)
                com_queue.put((h_pos, v_pos, dist))

                # this will make sure that, if there is an object found, the camera will stop the loop
                # and center on the object.
                calibrateAll()

            h_pos += 30
            horizontal_motor.run_to_abs_pos(position_sp=h_pos, speed_sp=200)
            time.sleep(0.5)


def shooter():
    # When performing a queue.get operation, the thread will be blocked if there are no objects in the queue

    y = 200 / 2
    if 150 > com_queue.get()[2] > 50:
        shooter_inclination_motor.run_to_abs_pos(position_sp=y + 22)
    elif 150 > com_queue.get()[2] > 100:
        shooter_inclination_motor.run_to_abs_pos(position_sp=y + 45)

    # TODO: implement shooter activation function
    shooter_inclination_motor.on_for_degrees(SpeedPercent(50), 360)


def calculate_inclination_angle(distance):
    initial_velocity = 23  # Constant speed
    return math.degrees(0.5 * math.asin((distance * 9.81) / (initial_velocity ** 2)))


def calibrateAll():
    # Pixy2 has a resolution of 328 * 200
    # make sure that the function will properly move the camera/shooter
    x = 328 / 2
    y = 200 / 2
    if x != com_queue.get()[0]:
        horizontal_motor.run_to_abs_pos(position_sp=x, speed_sp=200)
    if y != com_queue.get()[1]:
        vertical_motor.run_to_abs_pos(position_sp=y, speed_sp=200)


# Creating threads:
cam_thread_started = False
shooter_thread_started = False


def start_threads():
    global cam_thread_started
    global shooter_thread_started
    if not cam_thread_started:
        cam_thread_started = True
        threading.Thread(target=scan_and_detect).start()
        print("The cam detection thread has started")
    if not shooter_thread_started:
        threading.Thread(target=shooter).start()
        print("The shooter thread has started")


try:
    while True:
        cam_detect()

except KeyboardInterrupt:
    print("Stop!")
    sound.speak("Stopping")
    horizontal_motor.stop()
    vertical_motor.stop()
