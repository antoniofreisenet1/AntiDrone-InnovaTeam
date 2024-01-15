import time
from smbus import SMBus
from ev3dev2.port import LegoPort
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4

# Constants
CAMERA_PORT = INPUT_3  # Change according to your setup
I2C_ADDRESS = 0x54  # Default I2C address of PixyCam
SMBUS_PORT = 3  # Corresponds to INPUT_4

# Configure port for PixyCam
lego_port = LegoPort(CAMERA_PORT)
lego_port.mode = 'other-i2c'
time.sleep(0.5)

# Initialize SMBus
bus = SMBus(SMBUS_PORT)


# Function to read data from PixyCam
def read_pixy_data():
    # Request data packet
    request_data = [174, 193, 32, 2, 1, 1]  # Example request for signature 1
    bus.write_i2c_block_data(I2C_ADDRESS, 0, request_data)

    # Read block of data
    block = bus.read_i2c_block_data(I2C_ADDRESS, 0, 20)
    print(block)
    # Extract and print data
    if block[7] * 256 + block[6] == 1:  # Check for signature 1
        x = block[9] * 256 + block[8]
        y = block[11] * 256 + block[10]
        w = block[13] * 256 + block[12]
        h = block[15] * 256 + block[14]
        print("Detected object: X={}, Y={}, Width={}, Height={}".format(x, y, w, h))
    else:
        print("No object detected")


# Main loop
try:
    while True:
        read_pixy_data()
        time.sleep(1)
except KeyboardInterrupt:
    print("Script ended by user")
