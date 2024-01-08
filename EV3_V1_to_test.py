import math
import threading
import time
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, Motor
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.port import LegoPort
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sound import Sound
from smbus2 import SMBus

# Constants
CAMERA_PORT = INPUT_1
H_MOTOR_PORT = OUTPUT_A  # Horizontal motor for camera movement
V_CAMERA_MOTOR_PORT = OUTPUT_B  # Vertical motor for camera movement
V_SHOOTER_MOTOR_PORT = OUTPUT_C  # Vertical motor for shooter mechanism
SHOOT_MOTOR_PORT = OUTPUT_D  # Motor for shooting
TOUCH_SENSOR_PORT = INPUT_2
OBJ_WIDTH = 3  # Approx width of the object in cm
OBJ_HEIGHT = 4  # Approx height of the object in cm


class MotorController:
    def __init__(self, port, motor_range, max_speed=1000, motor_type=Motor, name=""):
        self.motor = motor_type(port)
        self.motor_range = motor_range  # Tuple (min, max) in degrees
        self.max_speed = max_speed
        self.name = name

    def __repr__(self):
        return "<MotorController name='{}'>".format(self.name)

    def move_to_position(self, position, speed_pct):
        """
        Moves the motor to a specified position within its range.
        :param position: Target position in degrees.
        :param speed_pct: Speed percentage.
        """
        # Ensure that the position is within the motor's range
        if position < self.motor_range[0]:
            position = self.motor_range[0]
        elif position > self.motor_range[1]:
            position = self.motor_range[1]

        # Move the motor
        self.motor.run_to_abs_pos(position_sp=position, speed_sp=200)
    def stop(self):
        """
        Stops the motor immediately.
        """
        self.motor.stop()


class Camera:
    def __init__(self, port, address=0x54, sigs=2):
        self.port = LegoPort(port)
        self.port.mode = "other-i2c"
        self.bus = SMBus(3)
        self.address = address
        self.sigs = sigs
        self.request_data = [174, 193, 32, 2, self.sigs, 1]  # Update as needed for your use case

        self.bus.write_i2c_block_data(self.address, 0, [174, 193, 14, 0])
        block = self.bus.read_i2c_block_data(self.address, 0, 13)
        print("Firmware version: {}.{}".format(block[8], block[9]))

    def detect_object(self):
        """
        Detect the object with the correct signature and return the position and distance.
        :param obj_width: Width of the object in some units.
        :param obj_height: Height of the object in the same units.
        :return: Tuple of (x, y, distance)
        """
        obj_width = OBJ_WIDTH
        obj_height = OBJ_HEIGHT
        # Request block
        self.bus.write_i2c_block_data(self.address, 0, self.request_data)

        # Read block request
        block = self.bus.read_i2c_block_data(self.address, 0, 20)

        # Extract positional and size information
        x = block[9] * 256 + block[8]
        y = block[11] * 256 + block[10]
        w = block[13] * 256 + block[12]
        h = block[15] * 256 + block[14]
        print("Detection : {}, {}, {}, {}".format(x, y, w, h))
        if x < 32895:  # Default value
            camera_fov = 60  # Field of view in degrees
            pixel_size = (w + h) / 2
            distance = (obj_width + obj_height) / (2 * pixel_size * math.tan(math.radians(camera_fov / 2)))
            return x, y, distance
        else:
            return None, None, None


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0

    def calculate(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.last_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output


def map_to_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def is_valid_distance(distance):
    return 150 > distance > 1


def scan_and_detect_1(camera, h_motor, v_motor):
    """
    Scan the area to detect the object. Once found, return its position.
    :return: Tuple of (h_pos, v_pos, distance) if object is found, else (None, None, None)
    """
    for v_pos in range(v_motor.motor_range[0], v_motor.motor_range[1] + 1, 10):  # Vertical scan
        v_motor.move_to_position(v_pos, 30)
        for h_pos in range(h_motor.motor_range[0], h_motor.motor_range[1] + 1, 30):  # Horizontal scan
            print("V_pos = {}, H_pos = {}".format(v_pos, h_pos))
            x, y, distance = camera.detect_object(OBJ_WIDTH, OBJ_HEIGHT)
            # print(x, y, distance)
            if (x, y, distance) == (None, None, None):
                continue
            if is_valid_distance(distance):
                return h_pos, v_pos, distance
    return None, None, None


def scan_and_detect(camera, h_motor, v_motor):
    # Same function as Antonio with slight modifications
    h_pos = h_motor.motor_range[0]
    print("start scan and detect")
    for v_pos in range(v_motor.motor_range[0], v_motor.motor_range[1] + 1, 10):  # Vertical scan
        v_motor.move_to_position(v_pos, 20)
        print("H Pos : {}, V Pos : {}".format(h_pos, v_pos))
        while h_pos < h_motor.motor_range[1]:  # Horizontal scan
            x, y, dist = camera.detect_object()
            print("X: {}, Y : {}, Distance : {}".format(x, y, dist))
            if 150 > dist > 1:
                sound.speak('Destruction')
                print("Object detected at:", dist, "Horizontal:", h_pos, "Vertical: ", v_pos)
                return h_pos, v_pos, dist

                # this will make sure that, if there is an object found, the camera will stop the loop
                # and center on the object.

            h_pos += 30
            h_motor.move_to_position(h_pos, 20)
            time.sleep(0.5)


def spiral_scan_and_detect(camera, h_motor, v_motor, sound, step_size=30):
    """
    Spiral scan to detect the object. Starts from the center and spirals outwards.
    :return: Tuple of (h_pos, v_pos, distance) if object is found, else (None, None, None)
    """
    h_center = (h_motor.motor_range[0] + h_motor.motor_range[1]) / 2
    v_center = (v_motor.motor_range[0] + v_motor.motor_range[1]) / 2

    h_motor.move_to_position(h_center, 30)
    v_motor.move_to_position(v_center, 30)

    for radius in range(0, max(h_motor.motor_range[1], v_motor.motor_range[1]), step_size):
        for h_pos in range(h_center - radius, h_center + radius + 1, step_size):
            for v_pos in [v_center - radius, v_center + radius]:
                h_motor.move_to_position(h_pos, 30)
                v_motor.move_to_position(v_pos, 30)
                sound.speak("Pause")
                time.sleep(0.5)
                x, y, distance = camera.detect_object(OBJ_WIDTH, OBJ_HEIGHT)
                if is_valid_distance(distance):
                    return h_pos, v_pos, distance

        for v_pos in range(v_center - radius, v_center + radius + 1, step_size):
            for h_pos in [h_center - radius, h_center + radius]:
                h_motor.move_to_position(h_pos, 30)
                v_motor.move_to_position(v_pos, 30)
                time.sleep(0.5)
                x, y, distance = camera.detect_object(OBJ_WIDTH, OBJ_HEIGHT)
                if is_valid_distance(distance):
                    return h_pos, v_pos, distance

    return None, None, None


def track_object(h_motor, v_motor, camera, pid_horizontal, pid_vertical):
    last_time = time.time()
    object_found = False

    while True:
        current_time = time.time()
        delta_time = current_time - last_time

        if not object_found:
            h_pos, v_pos, distance = scan_and_detect_1(camera, h_motor, v_motor)
            if h_pos is not None:
                object_found = True
                continue

        x, y, distance = camera.detect_object(OBJ_WIDTH, OBJ_HEIGHT)
        if x is None or y is None:
            object_found = False
            continue

        error_x = 160 - x  # Assuming 320 as camera width
        error_y = 120 - y  # Assuming 240 as camera height
        pid_output_x = pid_horizontal.calculate(error_x, delta_time)
        pid_output_y = pid_vertical.calculate(error_y, delta_time)

        h_motor.move_to_position(map_to_range(pid_output_x, -1000, 1000, 0, 1), 30)
        v_motor.move_to_position(map_to_range(pid_output_y, -1000, 1000, 0, 1), 30)

        last_time = current_time
        time.sleep(0.1)


def calculate_additional_inclination(dist):
    BALL_VELOCITY = 23  # Constant speed in m/s
    try:
        return math.degrees(0.5 * math.asin((dist * 9.81) / (BALL_VELOCITY ** 2)))
    except ValueError:
        return None


def aim_and_shoot(v_shooter_motor, shoot_motor, camera, target_distance, current_camera_inclination):
    additional_inclination = calculate_additional_inclination(target_distance)
    if additional_inclination is not None:
        total_inclination = current_camera_inclination + additional_inclination
        v_shooter_motor.move_to_position(total_inclination, 30)
        shoot_motor.shoot()


if __name__ == "__main__":
    sound = Sound()
    # Initialization
    h_motor = MotorController(H_MOTOR_PORT, (-379, 481), motor_type=LargeMotor, name="h_motor")
    v_camera_motor = MotorController(V_CAMERA_MOTOR_PORT, (-10, 44), name="v_cam_motor")
    v_shooter_motor = MotorController(V_SHOOTER_MOTOR_PORT, (-536, -274), name="v_shoot_motor")
    shoot_motor = MotorController(SHOOT_MOTOR_PORT, (0, 360), motor_type=LargeMotor, name="shoot_motor")
    camera = Camera(CAMERA_PORT)

    touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)

    update_pos = input("Update positions ? Y/N/B")
    if update_pos == "Y":
        for motor in (h_motor, v_camera_motor, v_shooter_motor):
            sound.speak("Press input button to configure ")
            if input("Put {} in start position and press Enter ->".format(motor.name)) == "":
                motor.motor_range[0] = motor.motor.position
                print("<MotorController name={}, index={}, position={}>".format(motor.name, 0, motor.motor.position))

            if input("Put {} in end position and press Enter ->".format(motor.name)) == "":
                motor.motor_range[1] = motor.motor.position
                print("<MotorController name={}, index={}, position={}>".format(motor.name, 1, motor.motor.position))

    elif update_pos == "B":
        # Initialize TouchSensor

        for motor in (h_motor, v_camera_motor, v_shooter_motor):
            sound.speak("Set {} to start position and press the touch sensor".format(motor.name))
            while not touch_sensor.is_pressed:  # Wait until TouchSensor is pressed
                time.sleep(0.01)  # Small delay to avoid busy waiting
            motor.motor_range[0] = motor.motor.position
            print("<MotorController name={}, index={}, position={}>".format(motor.name, 0, motor.motor.position))

            sound.speak("Set {} to end position and press the touch sensor".format(motor.name))
            while not touch_sensor.is_pressed:  # Wait until TouchSensor is pressed
                time.sleep(0.01)  # Small delay to avoid busy waiting
            motor.motor_range[1] = motor.motor.position
            print("<MotorController name={}, index={}, position={}>".format(motor.name, 1, motor.motor.position))

    # PID Controllers
    pid_horizontal = PIDController(0.4, 0.01, 0.05)
    pid_vertical = PIDController(0.4, 0.01, 0.05)

    # Main loop
    try:
        sound.speak("Starting scan")
        h_motor.move_to_position(0, 30)  # Move horizontal motor to start position
        v_camera_motor.move_to_position(0, 30)


        while True:
            for move in range(0, 180, 5):
                # h_motor.move_to_position(move, 30)
                h_motor.motor.run_to_abs_pos(position_sp=move, speed_sp=50)
                time.sleep(2)
            exit(0)
            # threading.Thread(target=scan_and_detect(camera,h_motor, v_camera_motor))
            # scan_result = scan_and_detect(camera, h_motor, v_camera_motor)
            # track_object(h_motor, v_camera_motor, camera, pid_horizontal, pid_vertical)
            # aim_and_shoot(v_shooter_motor, shoot_motor, camera, distance, v_camera_motor.motor.position)
            # if scan_result is not None:
            # h_pos, v_pos, distance = scan_result

            # else:
            # pass
            # Handle the case where no object is detected
            # print("No object detected.")
            # Add any additional logic you need here
    except touch_sensor.is_pressed:
        print("Interrupted by user")
        exit(0)
