from package.pid_controller import PIDController
from ev3dev2.sound import Sound


def test_pid_controller():
    pid = PIDController(0.4, 0.01, 0.05)
    print("Testing PIDController...")
    # Simulate error input and time delta
    error = 10
    delta_time = 0.1
    output = pid.calculate(error, delta_time)
    print(f"Output: {output}")


if __name__ == "__main__":
    test_pid_controller()
