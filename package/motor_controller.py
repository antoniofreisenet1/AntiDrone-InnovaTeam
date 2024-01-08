from ev3dev2.motor import MediumMotor, LargeMotor, SpeedPercent

class MotorController:
    def __init__(self, port, motor_range, max_speed=1000, motor_type=MediumMotor, name=""):
        self.motor = motor_type(port)
        self.motor_range = list(motor_range)
        self.max_speed = max_speed
        self.name = name

    def __repr__(self):
        return "<MotorController name='{}'>".format(self.name)

    def move_to_position(self, position, speed_pct):
        position_sp = self.motor_range[0] + position * (self.motor_range[1] - self.motor_range[0])
        self.motor.on_to_position(SpeedPercent(speed_pct), position_sp)

    def shoot(self):
        self.motor.on_for_degrees(SpeedPercent(100), 360)

# Test code for MotorController
if __name__ == "__main__":
    print("Testing MotorController...")
    # Add your test code here
