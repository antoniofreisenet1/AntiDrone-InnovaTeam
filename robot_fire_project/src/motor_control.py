import ev3dev.ev3 as ev3

class MotorControl:
    def __init__(self):
        self.motor = ev3.LargeMotor('outA')  # Adaptez à votre configuration

    def run_motor(self, position):
        # Exemple de contrôle de moteur
        self.motor.run_timed(time_sp=1000, speed_sp=position["azimuth"] * 100)
