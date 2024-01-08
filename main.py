import sched
import string
from time import *
from queue import Queue
from threading import Thread
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
sigs = 2

# I2C bus and address
bus = SMBus(3)
address = 0x54
# Define motors
horizontal_motor = LargeMotor(OUTPUT_A)  # Horizontal scan motor
vertical_motor = LargeMotor(OUTPUT_B)    # Vertical scan motor
shooter_Vmotor = LargeMotor(OUTPUT_C)   #Shooter aim motor
shooter_trigger = LargeMotor(OUTPUT_D) #Shooter trigger motor

# Define scan range (in degrees)
horizontal_range = [0, 360]  # Range for horizontal scanning
vertical_range = [45, 90]     # Range for vertical scanning

# Scheduler declaration: for multithreading functions

scheduler = sched.scheduler()

def motor_test():
    # Test both motors
    try:
        horizontal_motor.run_to_abs_pos(position_sp=0, speed_sp=100)
        vertical_motor.run_to_abs_pos(position_sp=0, speed_sp=100)

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
    # TODO: Remake the formula to better reflect the distance to an object.

    AVERAGE_GREEN_OBJECT_SIZE = 1000  # size in pixels
    green_object_size = w
    distance = AVERAGE_GREEN_OBJECT_SIZE / (2*green_object_size * math.tan(math.radians(60/2)))

    return [distance, x, y]


def scan_and_detect():
    for v_pos in range(vertical_range[0], vertical_range[1] + 1, 10):  # Vertical scan
        for h_pos in range(horizontal_range[0], horizontal_range[1] + 1, 30):  # Horizontal scan
            dist = cam_detect()
            if is_valid_distance(dist[0]):
                handle_detection(dist)
            horizontal_motor.run_to_abs_pos(position_sp = h_pos, speed_sp = 50)
        vertical_motor.run_to_abs_pos(position_sp=v_pos, speed_sp=200)


def is_valid_distance(distance):
    return 150 > distance > 1


def handle_detection(distance):
    sound.speak('Destruction')
    print("Object detected at:", distance[0], "Horizontal: ", distance[1], "Vertical: ",distance[1])
    #com_queue.put((horizontal, vertical, distance))
    calibrateAll(distance[1], distance[2])
    shooter(distance[0])


def calculate_inclination_angle(distance):
    initial_velocity = 23  # Constant speed
    return math.degrees(0.5 * math.asin((distance * 9.81) / (initial_velocity ** 2)))


def shooter(dist):
    # When performing a queue.get operation, the thread will be blocked if there are no objects in the queue

    y = 200/2
    pos = calculate_inclination_angle(dist)
    # For the code to work, we need to be constantly sending data to the queue (multithread only)
    shooter_Vmotor.run_to_abs_pos(position_sp = pos)
    shooter_trigger.run_timed(time_sp = 1, speed_sp = 200)

    # TODO: implement shooter activation function



def calibrateAll(hpos, vpos):
    # Pixy2 has a resolution of 328 * 200
    # make sure that the function will properly move the camera/shooter
    x = 328/2
    y = 200/2
    while x < hpos:
        horizontal_motor.run_direct(duty_cycle_sp = 20)
        sleep(0.1)
    while x >= hpos:
        horizontal_motor.run_direct(duty_cycle_sp = -20)
        sleep(0.1)
    while y <= vpos:
        vertical_motor.run_direct(duty_cycle_sp = 20)
        sleep(0.1)
    while y > vpos:
        vertical_motor.run_direct(duty_cycle_sp = -20)
        sleep(0.1)


# Input detection function:


def manual_overtake():
    """
    This function detects a command given by the user. If the command is correct, the robot will cease automatic mode
    and switch to manual mode, performing the command given. Once the command has been executed, or once the user types
    the command "RETURN", the robot will return to automatic mode.
    :param:
    :return:
    """
    command = input()
    aux = command.split(" ")
    print(aux)
    if(command == 'SHOOT'):
        shooter_trigger.run_timed(time_sp = 1, stop_action = 'coast')
    elif(command == 'SHUTDOWN'):
        print("Stop!")
        sound.speak("Stopping")
        horizontal_motor.stop()
        vertical_motor.stop()
        shooter_trigger.stop()
        shooter_Vmotor.stop()
        exit()
    elif(command == 'SPEAK'):
        if(cam.is_alive()):
            sound.speak("DETECTING")
        else:
            sound.speak("IDLING")
    elif(aux[0] == 'TURN'):
        if(aux[1] == 'RIGHT'):
            horizontal_motor.run_to_rel_pos(position_sp = aux[2])
        elif(aux[1] == 'LEFT'):
            horizontal_motor.run_to_rel_pos(position_sp = -aux[2])
    elif(command == 'CANCEL'):
        exit()


# Creating threads:
t = Thread(target = manual_overtake, name = "Thread_Input_Detect", daemon= True)
cam = Thread(target = scan_and_detect, name = "Thread_Cam")

try:

    motor_test()
    cam_setup()
    t.start()
    cam.start()

    #event1 = scheduler.enter(1, 1, manual_overtake)
    while True:
        scan_and_detect()



except KeyboardInterrupt:
    print("Stop!")
    sound.speak("Stopping")
    horizontal_motor.stop()
    vertical_motor.stop()
