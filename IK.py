import math
from my_teachMover import TeachMover

class my_InverseKinematics:
    # TeachMover2 Geometrical Paramaters (inches)
    H = 7.625   # The length of Base to Shoulder
    L = 7.0     # The length of Shoulder to Elbow and Elbow to Wrist
    LL = 3.8    # The length of Wrist

    C = 2 * math.pi     # Radians of a circle
    B_C = 7072 / C      # Base motor:             7072 steps in 1 rotation -> Base can only go 180 deg? why is it still be divided by 2*pi
    S_C = 7072 / C      # Shoulder motor:         7072 steps in 1 rotation
    E_C = 4158 / C      # Elbow motor:            4158 steps in 1 rotation
    W_C = 1536 / C      # Right/Left Wrist motor: 1536 steps in 1 rotation
    G_C = 2330 / C      # Gripper motor:          2330 steps in 1 rotation

    # initialize robot current coordinates
    def __init__(self, x=7, y=0, z=14.625):
        self.x = x
        self.y = y
        self.z = z
        return
    
    # Uses the robot's current coordinates to determine what the new stepper values after given change
    # FIXME: add the ability to control gripper angle based off palm vector
    def FindStep(self, dx, dy, dz, directionChange=0):
        self.x = dx
        self.y = dy
        self.z = dz
        print(f"Robot target coordinates - x: {self.x}, y: {self.y}, z: {self.z}")
        z0 = self.z - self.H
        Lxy = math.sqrt(self.x**2 + self.y**2)
        l1 = math.sqrt(Lxy**2 + z0**2) / 2
        # print(f"z0: {z0}, Lxy: {Lxy}, l1: {l1}")

        try:
            phi1 = math.acos(l1/self.L)
            phi3 = math.atan(z0/Lxy)
            theta1 = math.atan(self.y/self.x) # determines the step for the Base motor
            theta2 = phi1 + phi3 # determines step for the Shoulder motor
            theta3 = phi3 - phi1 # determines step for the Elbow motor
            print(f"phi1: {math.degrees(phi1)}, phi3: {math.degrees(phi3)}, theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}")
        except Exception as error:
            print("Math Domain Error")
            return 0,0,0,0,0
        
        step1 = int(theta1 * self.B_C) + 1768
        step2 = int((math.radians(90) - theta2) * self.S_C) + 1100
        step3 = int((math.radians(90) - theta3) * self.E_C) 
        print(f"IK results - step1: {step1}, step2: {step2}, step3: {step3}")
        return step1, step2, step3, 420, 0 # returning 0's for step4 and step5 so number of return values matches Zilin's IK implementation, makes switch btw implementations easier
    
    def getCoords(self):
        return self.x, self.y, self.z

    # update robots coordinates
    def incrCoords(self, x, y, z):
        self.x += x
        self.y += y
        self.z += z
    

if __name__ == "__main__":
    IK = my_InverseKinematics()

    # NOTE: Testing IK and TeachMover in combination
    robot = TeachMover('COM3')
    # the current steps for each motor must be initalized, this is my guesses for what the default position motors are
    # robot.set_motor_vals(1768, 1100, 1040, 0, 0, 0)
    robot.print_motors()

    j1, j2, j3, j4, j5 = IK.FindStep(0,0,2,0,0)
    # print(f"target motor steps: {j1} {j2} {j3} {j4} {j5}") # Default steps is 3536, 3536, 2079
    robot.set_step(200, j1, j2, j3, j4, j5, 0)

    # NOTE: Testing ranges for dx,dy,dz
    # dx, dy, dz = -7, -7, -7
    # while dx < 7:
    #     while dy < 7: 
    #         while dz < 7:
    #             print(f"dx: {dx}, dy: {dy}, dz: {dz}")
    #             IK.FindStep(dx,dy,dz,0,0)
    #             dz += 1
    #         dy += 1
    #         dz = -7
    #     dx += 1
    #     dy = -7
    #     dz = -7

    # dz limits determined by dx, dy, and curr position ->  can go high as almost 7
    # dx, dy, dz = 0, 0, -7
    # while dz < 7:
    #             print(f"dx: {dx}, dy: {dy}, dz: {dz}")
    #             IK.FindStep(dx,dy,dz,0,0)
    #             dz += 1


