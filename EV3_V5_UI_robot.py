# robot.py
import socket
import math
import sys
import time
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, Motor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor
from ev3dev2.sound import Sound
from smbus2 import SMBus
import threading

# Replace with your computer's IP address and chosen port
HOST = '172.20.10.2'
PORT = 8000

H_MOTOR_PORT = OUTPUT_A  # Horizontal motor for camera movement
V_CAMERA_MOTOR_PORT = OUTPUT_B  # Vertical motor for camera movement
V_SHOOTER_MOTOR_PORT = OUTPUT_C
TOUCH_SENSOR_PORT = INPUT_1
SHOOT_MOTOR_PORT = OUTPUT_D
ULTRASONIC_PORT = INPUT_2


class MotorController:
    """
    Class designated for all motor related functions. Contains gear ratios and the speed percentage.
    """
    speed_pct = 15
    GEAR_RATIOS = {
        "h_motor": 4.67,  # Replace with actual gear ratio
        "v_cam_motor": 1.67,  # Replace with actual gear ratio
        "v_shoot_motor": 3.34,  # Replace with actual gear ratio
        # Add other motors and their ratios as needed
    }

    def __init__(self, port, motor_range, max_speed=1000, motor_type=MediumMotor, name=""):
        self.motor = motor_type(port)
        self.motor_range = list(motor_range)
        self.max_speed = max_speed
        self.name = name
        self.gear_ratio = self.GEAR_RATIOS.get(name, 1.0)  # Default to 1.0 if name not found

    def in_range(self):
        """Function that tests if the motor position is in a valid range"""
        return self.motor.position > self.motor_range[1] or self.motor.position < self.motor_range[0]

    def reset(self):
        """Returns the motor to its original position"""
        self.motor.reset()

    def move_to_degrees(self, degrees):
        """Moves the motor to a specified position using degrees.
        :param: degrees
        """
        self.motor.run_to_abs_pos(position_sp=degrees, speed_sp=SpeedPercent(self.speed_pct))

    def run_for_degrees(self, degrees):
        """Makes a motor run for a certain amount of degrees.
        :paaram: degrees
        """
        # Calculate adjusted degrees based on gear ratio
        adjusted_degrees = degrees * self.gear_ratio
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), adjusted_degrees)

    def shoot(self):
        """Makes the motor shoot."""
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), 400)

    def get_position(self):
        """Gets the current posiiton of the motor"""
        # Replace this with the actual code to get motor positions from Mindstorms
        return int(self.motor.position / self.gear_ratio)  # Example values

    def run_full_turn(self):
        """Makes a 360º turn"""
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), 360)


def listen_for_commands(arguments):
    """Makes the robot listen for commands given by the user through the console.

    :param: arguments"""
    # add objects as arguments in the parenthesis to call them
    sock, sound, h_motor, v_camera_motor, shoot_motor, ultrasonic_sensor, touch_sensor = arguments
    print(shoot_motor)

    command_functions = {
        "run_full_turn": shoot_motor.run_full_turn,
        "disconnect": lambda: handle_disconnect(sock),
        "scan": lambda: scan(h_motor, v_camera_motor, touch_sensor, sound, ultrasonic_sensor, shoot_motor),
        "no_detection_scan": lambda: no_detection_scan(h_motor, v_camera_motor, touch_sensor, sound, ultrasonic_sensor, shoot_motor),
        # Map more commands to functions here
    }

    while True:
        try:
            data = sock.recv(1024).decode().strip()
            if data in command_functions:
                command_functions[data]()

        except BlockingIOError:
            # No data received
            pass
        except socket.error as e:
            # Handle other socket errors
            print("Socket error:", e)
            break


def calculate_inclination(dist):
    """Performs the necessary calculations to determine the inclination required to shoot at the target.

    :param: dist"""
    BALL_VELOCITY = 23  # Constant speed in m/s
    GRAVITY = 9.81  # Acceleration due to gravity in m/s^2
    HORIZONTAL_OFFSET = 5  # Horizontal offset in cm
    VERTICAL_OFFSET = 6  # Vertical offset in cm

    # Convert cm to m for calculation
    dist_m = dist / 100  # Convert distance from cm to m
    horizontal_offset_m = HORIZONTAL_OFFSET / 100  # Convert to m
    vertical_offset_m = VERTICAL_OFFSET / 100  # Convert to m

    # Adjust distance for horizontal offset
    adjusted_dist_m = math.sqrt(dist_m ** 2 + horizontal_offset_m ** 2)

    try:
        # Basic angle calculation
        basic_angle_rad = 0.5 * math.asin((adjusted_dist_m * GRAVITY) / (BALL_VELOCITY ** 2))
        basic_angle_deg = math.degrees(basic_angle_rad)

        # Adjust angle for vertical offset
        # Calculate the height difference at the basic angle
        height_diff = math.tan(basic_angle_rad) * adjusted_dist_m

        # Adjusted height difference
        adjusted_height_diff = height_diff - vertical_offset_m

        # Calculate the adjusted angle
        adjusted_angle_rad = math.atan(adjusted_height_diff / adjusted_dist_m)
        return math.degrees(adjusted_angle_rad)
    except ValueError:
        return None


def scan(h_motor, v_motor, touch_sensor, sound, ultrasonic_sensor, shoot_motor):
    """Object detection scan loop

    :param: h_motor, v_motor, touch_sensor, sound, ultrasonic_sensor, shoot_motor"""
    h_deg_increment = 30  # Increment for horizontal motor
    v_deg_increment = 15  # Increment for vertical motor

    h_motor.run_for_degrees(h_motor.motor_range[0] - h_motor.motor.position)
    v_motor.run_for_degrees(v_motor.motor_range[0] - v_motor.motor.position)

    while h_motor.motor.position < h_motor.motor_range[1]:
        h_motor.run_for_degrees(h_deg_increment)
        while v_motor.motor.position < v_motor.motor_range[1]:
            v_motor.run_for_degrees(v_deg_increment)
            distance = ultrasonic_sensor.distance_centimeters
            print(distance)
            if is_valid_distance(distance) or touch_sensor.is_pressed:
                sound.speak('Target detected')
                shoot_motor.run_for_degrees(calculate_inclination(distance))
                return
        v_motor.run_for_degrees(v_motor.motor_range[0] - v_motor.motor.position + 30)  # Reset vertical motor
        h_motor.run_for_degrees(h_deg_increment)  # Increment horizontal motor
    #h_motor.run_for_degrees(h_motor.motor_range[0] - h_motor.motor.position)
    h_motor.run_for_degrees(h_motor.motor.position - h_motor.motor_range[0])


def no_detection_scan(h_motor, v_motor, touch_sensor, sound, ultrasonic_sensor, shoot_motor):
    """Scan function to execute in case no objects are found.

    :param: h_motor, v_motor, touch_sensor, sound, ultrasonic_sensor, shoot_motor
    """
    h_deg_increment = 30  # Increment for horizontal motor
    v_deg_increment = 15  # Increment for vertical motor

    h_motor.run_for_degrees(h_motor.motor_range[0] - h_motor.motor.position)
    v_motor.run_for_degrees(v_motor.motor_range[0] - v_motor.motor.position)

    while h_motor.motor.position < h_motor.motor_range[1]:
        h_motor.run_for_degrees(h_deg_increment)
        while v_motor.motor.position < v_motor.motor_range[1]:
            v_motor.run_for_degrees(v_deg_increment)
            distance = ultrasonic_sensor.distance_centimeters
            print(distance)
            if touch_sensor.is_pressed:
                sound.speak('Button pressed')
                shoot_motor.run_for_degrees(calculate_inclination(distance))
                return
        v_motor.run_for_degrees(v_motor.motor_range[0] - v_motor.motor.position + 30)  # Reset vertical motor
        h_motor.run_for_degrees(h_deg_increment)  # Increment horizontal motor
    h_motor.run_for_degrees(h_motor.motor_range[0] - h_motor.motor.position)


def is_valid_distance(distance):
    """Determine if the distance to the object is between 1 and 150cm

    :param: distance"""
    if distance is None:
        return False
    else:
        return 150 > distance > 1


def handle_disconnect(s):
    """Robot disconnect handler"""
    s.close()
    print("Disconnected")
    sys.exit(0)


def main():
    sound = Sound()
    # Initialization
    h_motor = MotorController(H_MOTOR_PORT, (0, 841), motor_type=LargeMotor, name="h_motor")
    v_camera_motor = MotorController(V_CAMERA_MOTOR_PORT, (-50, 50), name="v_cam_motor")
    v_shooter_motor = MotorController(V_SHOOTER_MOTOR_PORT, (0, 300), name="v_shoot_motor")
    shoot_motor = MotorController(SHOOT_MOTOR_PORT, (0, 360), motor_type=LargeMotor, name="shoot_motor")
    ultrasonic_sensor = UltrasonicSensor(ULTRASONIC_PORT)

    touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
    h_motor.reset()
    v_camera_motor.reset()
    v_shooter_motor.reset()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.setblocking(False)  # Make the socket non-blocking
        # Start the listening thread
        arguments = (s, sound, h_motor, v_camera_motor, shoot_motor, ultrasonic_sensor, touch_sensor)
        threading.Thread(target=listen_for_commands, args=(arguments,), daemon=True).start()
        # You have to add the objects in args to listen for certain commands
        while not touch_sensor.is_pressed:
            motor1_pos, motor2_pos = h_motor.get_position(), v_camera_motor.get_position()
            distance = int(ultrasonic_sensor.distance_centimeters)
            print(calculate_inclination(distance))
            data_packet = "{},{},{}\n".format(
                motor1_pos,
                motor2_pos,
                distance)
            print(data_packet, calculate_inclination(distance))
            s.sendall(data_packet.encode())

            time.sleep(0.1)  # Adjust as needed


if __name__ == '__main__':
    main()
