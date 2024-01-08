import math
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
    speed_pct = 30
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
        self.motor.run_to_abs_pos(position_sp=degrees, speed_sp=50)

    def run_for_degrees(self, degrees):
        # Calculate adjusted degrees based on gear ratio
        adjusted_degrees = degrees * self.gear_ratio
        self.motor.on_for_degrees(SpeedPercent(self.speed_pct), adjusted_degrees)


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
        # print("Detection : {}, {}, {}, {}".format(x, y, w, h))
        if x < 32895:  # Default value
            camera_fov = 60  # Field of view in degrees
            pixel_size = (w + h) / 2
            distance = (obj_width + obj_height) / (2 * pixel_size * math.tan(math.radians(camera_fov / 2)))
            print("Detection : X{}, Y{}, D{}".format(x, y, distance))
            return x, y, distance
        else:
            return None, None, None


def is_valid_distance(distance):
    if distance is None:
        return False
    else:
        return 150 > distance > 1


def scan_with_camera1(h_motor, v_motor, touch_sensor, sound, camera):
    h_deg_increment = 30  # Increment for horizontal motor
    v_deg_increment = 15  # Increment for vertical motor

    h_motor.move_to_degrees(h_motor.motor_range[0])
    v_motor.move_to_degrees(v_motor.motor_range[0])
    while h_motor.motor.position <= h_motor.motor_range[1]:
        h_motor.run_for_degrees(h_deg_increment)
        print(h_motor.motor.position, h_motor.motor_range[1])
        v_motor.move_to_degrees(v_motor.motor_range[0])
        while v_motor.motor.position <= v_motor.motor_range[1]:
            v_motor.run_for_degrees(v_deg_increment)
            print(v_motor.motor.position, v_motor.motor_range[1])
            x, y, distance = camera.detect_object()
            if is_valid_distance(distance) or touch_sensor.is_pressed:
                sound.speak('Target detected or program has been stopped')
                return


def scan_with_camera(h_motor, v_motor, touch_sensor, sound, camera):
    h_deg_increment = 30  # Increment for horizontal motor
    v_deg_increment = 15  # Increment for vertical motor

    h_motor.run_for_degrees(h_motor.motor_range[0] - h_motor.motor.position)
    v_motor.run_for_degrees(v_motor.motor_range[0] - v_motor.motor.position)

    while h_motor.motor.position < h_motor.motor_range[1]:
        h_motor.run_for_degrees(h_deg_increment)
        while v_motor.motor.position < v_motor.motor_range[1]:
            v_motor.run_for_degrees(v_deg_increment)
            x, y, distance = camera.detect_object()
            if is_valid_distance(distance) or touch_sensor.is_pressed:
                sound.speak('Target detected or program has been stopped')
                return
        v_motor.run_for_degrees(v_motor.motor_range[0] - v_motor.motor.position)  # Reset vertical motor
        h_motor.run_for_degrees(h_deg_increment)  # Increment horizontal motor


if __name__ == "__main__":
    sound = Sound()
    # Initialization
    h_motor = MotorController(H_MOTOR_PORT, (0, 841), motor_type=LargeMotor, name="h_motor")
    v_camera_motor = MotorController(V_CAMERA_MOTOR_PORT, (0, 150), name="v_cam_motor")
    v_shooter_motor = MotorController(V_SHOOTER_MOTOR_PORT, (-536, -274), name="v_shoot_motor")
    shoot_motor = MotorController(SHOOT_MOTOR_PORT, (0, 360), motor_type=LargeMotor, name="shoot_motor")
    camera = Camera(CAMERA_PORT)

    h_motor.reset()
    v_camera_motor.reset()
    v_shooter_motor.reset()
    shoot_motor.reset()

    touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
    # update_pos = input("Update positions ? Y/N")
    update_pos = "N"
    if update_pos == "Y":
        # Initialize TouchSensor
        for motor in [v_shooter_motor]:
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

    # Main loop
    try:
        sound.speak("Starting scan")
        scan_with_camera(h_motor, v_camera_motor, touch_sensor, sound, camera)
        # for h_deg in range(0, 180, 10):
        # h_motor.run_for_degrees(h_deg)
        # print(h_deg, h_motor.motor_range[1])

    except touch_sensor.is_pressed:
        print("Interrupted by user")
        exit(0)
