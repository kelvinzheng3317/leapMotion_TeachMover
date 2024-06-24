import serial
import sys
import time
import threading

# sys.path.append(r'C:\Users\Kelvin\Desktop\figures')
# try:
#     import _leapc_cffi
#     print("Module imported successfully.")
# except ImportError as e:
#     print(f"Failed to import module: {e}")

import leap
from IK_Zilin import InverseKinematics

class TeachMover:
    m1, m2, m3, m4, m5, m6 = 0, 0, 0, 0, 0, 0

    def __init__(self, portID: str, baudrate = 9600):
        try:
            self.con=serial.Serial(portID,baudrate,timeout=3)
            print("Success")

        except serial.SerialException as e:
            print("Error")
            return
        self.lock = threading.Lock()

    def send_cmd(self, cmd:str):
        if not cmd.endswith("\r"):
            cmd += "\r"

        # Attempt at threading ver
        # def msg_robot():
        #     with self.lock:
        #         self.con.write(cmd.encode())
        #         return self.con.readline().decode().strip()

        # thread = threading.Thread(target=msg_robot)

        # print(response)
        self.con.write(cmd.encode())
        response = self.con.readline().decode().strip()
        return response
    
    def move(self, spd, j1, j2, j3, j4, j5, j6):
        print(f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+j3}")
        # self.update_motors(j1, j2, j3, j4+j5, j4-j5, j6+j3)
        self.update_motors(j1, j2, j3, j4, j5, j6)
        response = self.send_cmd(f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+j3}")
        return response
    
    def set_step(self, spd, j1, j2, j3, j4, j5, j6):
        print(f"Moving directly to {j1} {j2} {j3} {j4} {j5} {j6}")
        diff1 = j1 - self.m1
        diff2 = j2 - self.m2
        diff3 = j3 - self.m3
        diff4 = j4 - self.m4
        diff5 = j5 - self.m5
        diff6 = j6 - self.m6
        self.update_motors(diff1, diff2, diff3, diff4, diff5, diff6)
        print(f"@STEPS {spd} {diff1}, {diff2}, {diff3}, {diff4}, {diff5}, {diff6}")
        response = self.send_cmd(f"@STEP {spd}, {diff1}, {diff2}, {diff3}, {diff4}, {diff5}, {diff6}")
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
        ret = self.move(240, -self.m1, -self.m2, -self.m3, -self.m4, -self.m5, -self.m6)
        return ret

    # Resets what the robot considers its "0 Position"
    def reset(self):
        print("Resetting 0 position")
        self.m1, self.m2, self.m3, self.m4, self.m5, self.m6 = 0, 0, 0, 0, 0, 0
        return self.send_cmd("@RESET")
    
    def read_pos(self):
        print("Reading position")
        pos = self.send_cmd("@READ")
        #Strip the leading status code
        return pos

    def readPosition(self):
        ret = self.send_cmd("@READ")
        #Strip the leading status code
        ret = ret.split('\r')[-1]
        return ret
    
    def close_grip(self):
        print("Closing grip")
        # TODO: UPDATE ROBOT MOTOR VALUES
        # THE LINE BELOW CLOSES BUT ITS VERY SLOW SO I'M USING A MOVE INSTEAD
        # self.send_cmd("@CLOSE")
        self.move(240, 0, 0, 0, 0, 0, -500)

    def open_grip(self):
        print("Opening grip")
        self.move(240, 0, 0, 0, 0, 0, 400)

    def test_thread(self, num):
        def msg_robot():
            with self.lock:
                print(f"starting msg {num}")
                time.sleep(0.5)
                print(f"finish msg {num}")

        thread = threading.Thread(target=msg_robot)
        thread.start()
        # print(response)
        return
            


if __name__ == "__main__":
    # INVERSE KINEMATICS TESTING
    # IK = InverseKinematics(0,0,0,0,0)
    # j1, j2, j3, j4, j5 = IK.FindStep(10, 2, 12, 0, 0)
    # print(f"results: {j1}, {j2}, {j3}, {j4}, {j5}")

    robot = TeachMover('COM3')

    # NOTE: THREAD TESTING
    # robot.test_thread(1)
    # for i in range(10):
    #     robot.test_thread(i)
    # time.sleep(10)

    # NOTE: GRIPPER TEST
    # robot.open_grip()
    # robot.close_grip()
    # robot.print_motors()

    # NOTE: MOVE_TO_SET_STEP TEST
    robot.move(240, 200, -150, 40, -150, -60, 30)
    robot.print_motors()
    time.sleep(1)
    robot.set_step(240,0,0,0,0,0,0)
    robot.print_motors()
    # time.sleep(0.5)
    # robot.returnToZero()

    # NOTE: @READ TESTING
    # pos = robot.readPosition()
    # print(type(pos))
    # print(pos)

    # robot.reset()
    # robot.print_motors()
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