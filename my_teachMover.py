import serial
import sys
import time

# sys.path.append(r'C:\Users\Kelvin\Desktop\figures')
# try:
#     import _leapc_cffi
#     print("Module imported successfully.")
# except ImportError as e:
#     print(f"Failed to import module: {e}")

import leap

class TeachMover:
    def __init__(self, portID: str, baudrate = 9600):
        try:
            self.con=serial.Serial(portID,baudrate,timeout=3)
            print("Success")

        except serial.SerialException as e:
            print("Error")

    def send_cmd(self, cmd:str):
        if not cmd.endswith("\r"):
            cmd += "\r"
        self.con.write(cmd.encode())
        reponse = self.con.readline().decode().strip()
        return reponse
    
    def move(self, spd, j1, j2, j3, j4, j5, j6):
        print(f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+j3}")
        response = self.send_cmd(f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+j3}")
        return response

    # FIXME: THIS CURRENTLY DOESN'T SEEM TO BE WORKING -> MAYBE @RESET DOESN'T ACTUALLY RESETS THE POSITION BUT RESETS ROBOT MEMORY??
    def reset(self):
        print("Resetting position")
        return self.send_cmd("@RESET")
    
    def read_pos(self):
        print("Reading position")
        pos = self.send_cmd("@READ")
        #Strip the leading status code
        return pos
    
    def close_grip(self):
        print("Closing grip")
        self.send_cmd("@CLOSE")


if __name__ == "__main__":
    robot = TeachMover('COM3')
    robot.move(200, 0, 0, 100, 0, 0, 0)
    time.sleep(0.5)
    # robot.reset()
    print("done")

# ser = serial.Serial()
# ser.port = 'COM3'
# ser.baudrate = 9600
# ser.timeout = 5.0
# print(f'Serial port is open: #{ser.is_open}')
# print(ser.name)
# ser.write(b'@STEP100,10,100,100,100,100,100')
# ser.write(b'@READ')
# print(ser.read(10))
# ser.close()