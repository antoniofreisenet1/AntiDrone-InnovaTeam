from camera import Camera
from object_detection import ObjectDetector
from calculations import calculate_position
from motor_control import MotorControl

def main():
    cam = Camera()
    detector = ObjectDetector()
    motor = MotorControl()

    while True:
        frame = cam.get_frame()
        objects = detector.detect_objects(frame)
        for obj in objects:
            position = calculate_position(obj)
            print(f"Objet: {obj['name']}, Position: {position}")
            motor.run_motor(position)

if __name__ == "__main__":
    main()
