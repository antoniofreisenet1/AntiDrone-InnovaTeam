from motor_controller import MotorController
from camera import Camera
from pid_controller import PIDController
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2

from utils import map_to_range, is_valid_distance
# Constants
CAMERA_PORT = INPUT_1
H_MOTOR_PORT = OUTPUT_A  # Horizontal motor for camera movement
V_CAMERA_MOTOR_PORT = OUTPUT_B  # Vertical motor for camera movement
V_SHOOTER_MOTOR_PORT = OUTPUT_C  # Vertical motor for shooter mechanism
SHOOT_MOTOR_PORT = OUTPUT_D  # Motor for shooting
TOUCH_SENSOR_PORT = INPUT_2
OBJ_WIDTH = 3  # Approx width of the object in cm
OBJ_HEIGHT = 4  # Approx height of the object in cm

# Initialization
h_motor = MotorController(H_MOTOR_PORT, (-379, 481), motor_type=LargeMotor, name="h_motor")
v_camera_motor = MotorController(V_CAMERA_MOTOR_PORT, (-10, 44), name="v_cam_motor")
v_shooter_motor = MotorController(V_SHOOTER_MOTOR_PORT, (-536, -274), name="v_shoot_motor")
shoot_motor = MotorController(SHOOT_MOTOR_PORT, (0, 360), motor_type=LargeMotor, name="shoot_motor")
camera = Camera(CAMERA_PORT)
pid_horizontal = PIDController(0.4, 0.01, 0.05)
pid_vertical = PIDController(0.4, 0.01, 0.05)

# Main loop
if __name__ == "__main__":
    while True:
        move = int(input("Give the moving number :"))
        h_motor.move_to_position(move, 30)
