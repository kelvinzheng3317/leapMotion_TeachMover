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
    m1, m2, m3, m4, m5, m6 = 0, 0, 0, 0, 0, 0

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
        response = self.con.readline().decode().strip()
        # print(response)
        return response
    
    def move(self, spd, j1, j2, j3, j4, j5, j6):
        print(f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+j3}")
        # self.update_motors(j1, j2, j3, j4+j5, j4-j5, j6+j3)
        self.update_motors(j1, j2, j3, j4, j5, j6)
        response = self.send_cmd(f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+j3}")
        return response

    def update_motors(self, j1, j2, j3, j4, j5, j6):
        self.m1 += j1
        self.m2 += j2
        self.m3 += j3
        self.m4 += j4
        self.m5 += j5
        self.m6 += j6
        print("New updated ", end="")
        self.print_motors()
    
    def print_motors(self):
        print(f"step motors: {self.m1}, {self.m2}, {self.m3}, {self.m4}, {self.m5}, {self.m6}")
    
    def returnToZero(self):
        # currentPos = self.readPosition().split(',')

        # speed = 240
        # j1 = -int(currentPos[0])
        # j2 = -int(currentPos[1])
        # j3 = -int(currentPos[2])
        # j4 = -int(currentPos[3])
        # j5 = -int(currentPos[4])
        # j6 = -int(currentPos[5])-j3
        print("returning to zero position")
        # ret = self.move(240, -self.m1, -self.m2, -self.m3, -0.5*(self.m4+self.m5), -0.5*(self.m4-self.m5), -self.m6-self.m3)
        ret = self.move(240, -self.m1, -self.m2, -self.m3, -self.m4, -self.m5, -self.m6-self.m3)
        return ret

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
        # TODO: UPDATE ROBOT MOTOR VALUES
        self.send_cmd("@CLOSE")

    def readPosition(self):
        ret = self.send_cmd("@READ")
        #Strip the leading status code
        ret = ret.split('\r')[-1]
        return ret


if __name__ == "__main__":
    robot = TeachMover('COM3')
    robot.print_motors()
    robot.move(240, 100, -50, 40, 50, 10, 30)
    time.sleep(0.5)
    robot.returnToZero()
    # pos = robot.readPosition()
    # print(type(pos))
    # print(pos)
    # robot.reset()
    print("Done")

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