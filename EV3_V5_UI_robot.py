# robot.py
import socket
import math
import time
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, Motor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sound import Sound
from smbus2 import SMBus
import threading

# Replace with your computer's IP address and chosen port
HOST = '172.20.10.2'
PORT = 8000

H_MOTOR_PORT = OUTPUT_A  # Horizontal motor for camera movement
V_CAMERA_MOTOR_PORT = OUTPUT_B  # Vertical motor for camera movement
TOUCH_SENSOR_PORT = INPUT_1
SHOOT_MOTOR_PORT = OUTPUT_D


class MotorController:
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
        return self.motor.position > self.motor_range[1] or self.motor.position < self.motor_range[0]

    def reset(self):
        self.motor.reset()

    def move_to_degrees(self, degrees):
        self.motor.run_to_abs_pos(position_sp=degrees, speed_sp=SpeedPercent(self.speed_pct))

    def run_for_degrees(self, degrees):
        # Calculate adjusted degrees based on gear ratio
        adjusted_degrees = degrees * self.gear_ratio
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), adjusted_degrees)

    def shoot(self):
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), 400)

    def get_position(self):
        # Replace this with the actual code to get motor positions from Mindstorms
        return int(self.motor.position / self.gear_ratio)  # Example values

    def run_full_turn(self):
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), 360)


def listen_for_commands(sock, shoot_motor):
    # add objects as arguments in the parenthesis to call them
    command_functions = {
        "run_full_turn": shoot_motor.run_full_turn,

        # Map more commands to functions here
    }

    while True:
        try:
            data = sock.recv(1024).decode().strip()
            if data in command_functions:
                command_functions[data]()
            if data == "scan":
                pass
                # Call the scanning functions here for now and delete the pass
        except BlockingIOError:
            # No data received
            pass
        except socket.error:
            # Handle other potential socket errors
            pass


def main():
    sound = Sound()
    # Initialization
    h_motor = MotorController(H_MOTOR_PORT, (0, 841), motor_type=LargeMotor, name="h_motor")
    v_camera_motor = MotorController(V_CAMERA_MOTOR_PORT, (0, 100), name="v_cam_motor")
    shoot_motor = MotorController(SHOOT_MOTOR_PORT, (0, 360), motor_type=LargeMotor, name="shoot_motor")

    touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
    h_motor.reset()
    v_camera_motor.reset()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.setblocking(False)  # Make the socket non-blocking
        # Start the listening thread
        threading.Thread(target=listen_for_commands, args=(s, shoot_motor), daemon=True).start()
        # You have to add the objects in args to listen for certain commands
        while not touch_sensor.is_pressed:
            motor1_pos, motor2_pos = h_motor.get_position(), v_camera_motor.get_position()
            s.sendall("{},{}\n".format(motor1_pos, motor2_pos).encode())
            time.sleep(0.1)  # Adjust as needed


if __name__ == '__main__':
    main()
