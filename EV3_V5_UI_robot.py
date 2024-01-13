import math
import socket
import time
import threading
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, Motor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor
from ev3dev2.sound import Sound

# Replace with your computer's IP address and chosen port
HOST = '172.20.10.2'
PORT = 8000

ULTRASONIC_MOTOR_PORT = OUTPUT_A  # Moteur pour le mouvement du capteur ultrasonique
SHOOT_MOTOR_PORT = OUTPUT_D

class MotorController:
    speed_pct = 15
    GEAR_RATIOS = {
         "h_motor": 4.67,  # Replace with actual gear ratio
        "ultrasonic_motor": 4.67,  # Replace with actual gear ratio
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

        # Initialize the motor at angle 0
        self.reset()

    def move_ultrasonic_motor(self, degrees):
        # Move the ultrasonic motor
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), degrees)

    def reset(self):
        # Reset the motor to angle 0
        self.motor.position = 0

    def move_to_degrees(self, degrees):
        # Adjust the movement direction based on the sign of degrees
        direction = math.copysign(1, degrees)
        adjusted_degrees = direction * degrees * self.gear_ratio
        self.motor.run_to_abs_pos(position_sp=adjusted_degrees, speed_sp=SpeedPercent(self.speed_pct))

    def shoot(self, distance):
        shooting_angle = self.calculate_shooting_angle(distance)
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), shooting_angle)

    def get_position(self):
        # Replace this with the actual code to get motor positions from Mindstorms
        return int(self.motor.position / self.gear_ratio)  # Example values

    def run_full_turn(self):
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), 360)

    def calculate_shooting_angle(self, distance):
        # Shooting angle calculation based on distance, sensor position, and shooting point position
        max_distance = 100  # Adjust as needed
        max_shooting_angle = 720  # Adjust as needed

        adjusted_distance = distance - 18  # Sensor is 18 cm in front
        shooting_angle = (adjusted_distance / max_distance) * max_shooting_angle

        shooting_angle -= math.degrees(math.atan2(9, adjusted_distance))  # Sensor is 9 cm down

        # Adjust shooting angle for the shooting point position
        shooting_angle += math.degrees(math.atan2(3, 13))  # Shooting point is 3 cm up and 13 cm in front

        shooting_angle = min(max_shooting_angle, max(0, shooting_angle))
        return shooting_angle

def get_distance_from_sensor(port):
    ultrasonic_sensor = UltrasonicSensor(port)
    return ultrasonic_sensor.distance_centimeters

def listen_for_commands(sock, shoot_motor):
    command_functions = {
        "run_full_turn": shoot_motor.run_full_turn,
        "shoot": lambda: shoot_with_distance(shoot_motor),
        # Add more commands as needed
    }

    while True:
        try:
            data = sock.recv(1024).decode().strip()
            if data in command_functions:
                command_functions[data]()
            if data == "scan":
                # Call the scanning functions here
                pass
        except BlockingIOError:
            # No data received
            pass
        except socket.error:
            # Handle other potential socket errors
            pass

def shoot_with_distance(shoot_motor):
    distance = get_distance_from_sensor(ULTRASONIC_SENSOR_PORT)
    shoot_motor.shoot(distance)

def main():
    sound = Sound()
    # Initialization
    ultrasonic_motor = MotorController(ULTRASONIC_MOTOR_PORT, (0, 841), motor_type=LargeMotor, name="ultrasonic_motor")
    shoot_motor = MotorController(SHOOT_MOTOR_PORT, (0, 360), motor_type=LargeMotor, name="shoot_motor")

    touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
    ultrasonic_motor.reset()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.setblocking(False)  # Make the socket non-blocking
        threading.Thread(target=listen_for_commands, args=(s, shoot_motor), daemon=True).start()
        while not touch_sensor.is_pressed:
            motor1_pos, motor2_pos = ultrasonic_motor.get_position(), ultrasonic_motor.get_position()
            s.sendall("{},{}\n".format(motor1_pos, motor2_pos).encode())
            time.sleep(0.1)  # Adjust as needed

if __name__ == '__main__':
    main()
