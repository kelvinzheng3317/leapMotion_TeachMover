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
    m1, m2, m3, m4, m5, m6 = 1768, 1100, 1040, 420, 0, 750

    def __init__(self, portID: str, baudrate = 9600):
        try:
            self.con=serial.Serial(portID,baudrate,timeout=4)
            print("Success")

        except serial.SerialException as e:
            print("Error")
            return
        # Sets motor step attributes to starting position values
        self.m1 = 1768
        self.m2 = 1100
        self.m3 = 1040
        self.m4 = 420
        self.m5 = 0
        self.m6 = 750
        self.lock = threading.Lock()

    def send_cmd(self, cmd:str):
        if not cmd.endswith("\r"):
            cmd += "\r"

        # Attempt at threading ver
        def msg_robot(cmd):
            with self.lock:
                self.con.write(cmd.encode())
                return self.con.readline().decode().strip()
        
        if self.lock.locked():
                print("Currently locked")
                return "locked"
        
        thread = threading.Thread(target=msg_robot, args=[cmd])
        response = thread.start()
        # FIXME: send_cmd will never return a response from the robot since the function will terminate before the thread does
        # print(response)
        # self.con.write(cmd.encode())
        # response = self.con.readline().decode().strip()
        return response
    
    def move(self, spd, j1, j2, j3, j4, j5, j6):
        # self.update_motors(j1, j2, j3, j4+j5, j4-j5, j6+j3)
        gripper_adj = 0
        if j3 > 0:
            gripper_adj = int(0.5*j3)
        elif j3 < 0:
            gripper_adj = int(0.25*j3)
        cmd = f"@STEP {spd}, {j1}, {j2}, {j3}, {j4+j5}, {j4-j5}, {j6+gripper_adj}"
        response = self.send_cmd(cmd)
        if response != "locked":
            print(cmd)
            self.increment_motors(j1, j2, j3, j4+j5, j4-j5, j6+gripper_adj)
        return response
    
    def set_step(self, spd, j1, j2, j3, j4, j5, j6):
        # prevent overextending the robot
        if (j1>7072 or j1<0 or j2>7072 or j2<0 or j3>4158 or j3<0 or j4>1536 or j4<0 or j5>1536 or j5<0 or j6>2330 or j6<0):
            print("Given motor step values are out of range")
            return
        gripper_adj = 0
        diff1 = j1 - self.m1
        diff2 = j2 - self.m2
        diff3 = j3 - self.m3
        diff4 = j4 - self.m4
        diff5 = j5 - self.m5
        diff6 = j6 - self.m6
        if diff3 > 0:
            gripper_adj = int(0.5*diff3)
        elif diff3 < 0:
            gripper_adj = int(0.5*diff3)
        # FIXME: diff2 causes the gripper to open and close
        cmd = f"@STEP {spd}, {diff1}, {diff2}, {diff3}, {diff4+diff5}, {diff4-diff5}, {diff6+gripper_adj}"
        response = self.send_cmd(cmd)
        if response != "locked":
            print(f"Going to motor steps {j1} {j2} {j3} {j4} {j5} {j6}")
            self.increment_motors(diff1, diff2, diff3, diff4+diff5, diff4-diff5, diff6+gripper_adj)
            print(cmd)
        return response
    
    def returnToStart(self):
        '''
        currentPos = self.readPosition().split(',')
        j1 = -int(currentPos[0])
        j2 = -int(currentPos[1])
        j3 = -int(currentPos[2])
        j4 = -int(currentPos[3])
        j5 = -int(currentPos[4])
        j6 = -int(currentPos[5])-j3
        '''
        # ret = self.move(240, 1768-self.m1, 1100-self.m2, 1040-self.m3, -self.m4, -self.m5, 750-self.m6)
        cmd = f"@STEP 240, {1768-self.m1}, {1100-self.m2}, {1040-self.m3}, {420-self.m4}, {-self.m5}, {750-self.m6}"
        response = self.send_cmd(cmd)
        if response != "locked":
            print("returning to zero position")
            self.increment_motors(1768-self.m1, 1100-self.m2, 1040-self.m3, 420-self.m4, -self.m5, 750-self.m6)
            print(cmd)
        return response

    def increment_motors(self, j1, j2, j3, j4, j5, j6):
        self.m1 += j1
        self.m2 += j2
        self.m3 += j3
        self.m4 += j4
        self.m5 += j5
        self.m6 += j6
        print("New updated ", end="")
        self.print_motors()

    def set_motor_vals(self, m1, m2, m3, m4, m5, m6):
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.m4 = m4
        self.m5 = m5
        self.m6 = m6
    
    def print_motors(self):
        print(f"step motors: {self.m1}, {self.m2}, {self.m3}, {self.m4}, {self.m5}, {self.m6}")

    # Resets what the robot considers its "0 Position"
    def reset(self):
        print("Resetting 0 position")
        self.m1, self.m2, self.m3, self.m4, self.m5, self.m6 = 0, 0, 0, 0, 0, 0
        return self.send_cmd("@RESET")
    
    def read_pos(self):
        print("Reading position")
        cmd = "@READ"
        self.con.write(cmd.encode())
        response = self.con.readline().decode().strip()
        #Strip the leading status code
        return response

    def readPosition(self):
        ret = self.send_cmd("@READ")
        #Strip the leading status code
        ret = ret.split('\r')[-1]
        return ret
    
    def close_grip(self):
        print("Closing grip")
        # TODO: UPDATE ROBOT MOTOR VALUES
        # @CLOSE IS VERY SLOW SO I'M USING A MOVE INSTEAD
        # self.send_cmd("@CLOSE")
        self.move(240, 0, 0, 0, 0, 0, -self.m6)

    def open_grip(self):
        if self.m6 < 1000:
            print("Opening grip")
            self.move(240, 0, 0, 0, 0, 0, 1000-self.m6)
        else:
            print("Grip already open")

    def lock_wait(self):
        while self.lock.locked():
            continue
        return

    counter = 0
    def test_thread(self, num):
        def msg_robot():
            if self.lock.locked():
                print(f"Currently locked for msg {num}")
                return

            with self.lock:
                print(f"starting msg {num}")
                self.counter += 1
                time.sleep(0.2)
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
    # response = robot.read_pos()
    # print(response)
    # robot.open_grip()
    # robot.move(240, 0,0,0,0,0,0)
    robot.set_step(240, 1768, 1100, 1040, 420, 400, 750)
    # robot.lock_wait()
    # robot.open_grip()
    robot.lock_wait()
    # # robot.move(240, 0,0,0,-400,0,0)
    robot.returnToStart()

    # NOTE: BASIC THREAD TESTING
    # robot.test_thread(1)
    # for i in range(10):
    #     print(f"Iteration {i}")
    #     robot.test_thread(i)
    #     time.sleep(0.1)
    # time.sleep(5)
    # print(robot.counter)

    # NOTE: ROBOT MOVE THREAD TESTING
    # i = 0
    # while True:
    #     print(f"iteration {i}")
    #     robot.move(200,100,10,0,0,0,0)
    #     time.sleep(0.1)
    #     i += 1

    # NOTE: GRIPPER TEST
    # robot.open_grip()
    # robot.close_grip()
    # robot.print_motors()

    # NOTE: MOVE_TO_SET_STEP TEST
    # robot.move(240, 200, -150, 40, -150, -60, 30)
    # robot.print_motors()
    # time.sleep(1)
    # robot.set_step(240,0,0,0,0,0,0)
    # robot.print_motors()

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