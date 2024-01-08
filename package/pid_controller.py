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

# Test code for PIDController
if __name__ == "__main__":
    print("Testing PIDController...")
    # Add your test code here
