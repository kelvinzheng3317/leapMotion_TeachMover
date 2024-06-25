import math

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
    
    # FIXME: add the ability to control gripper angle based off palm vector
    def FindStep(self, dx, dy, dz, dlw, drw):
        x = self.x + dx
        y = self.y + dy
        z = self.z + dz
        print(f"x: {x}, y: {y}, z: {z}")
        z0 = z - self.H
        Lxy = math.sqrt(x**2 + y**2)
        l1 = math.sqrt(Lxy**2 + z0**2) / 2
        print(f"z0: {z0}, Lxy: {Lxy}, l1: {l1}")

        try:
            phi1 = math.acos(l1/self.L)
            phi3 = math.atan(z0/Lxy)
            theta1 = math.atan(y/x) # determines the step for the Base motor
            theta2 = phi1 + phi3 # determines step for the Shoulder motor
            theta3 = phi3 - phi1 # determines step for the Elbow motor
            print(f"phi1: {math.degrees(phi1)}, phi3: {math.degrees(phi3)}, theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}")
        except Exception as error:
            print("Out of robot's effective range")
            return 0,0,0,0,0
        
        step1 = int(theta1 * self.B_C)
        step2 = int(theta2 * self.S_C)
        step3 = int(theta3 * self.E_C)
        print(f"NO ERROR - step1: {step1}, step2: {step2}, step3: {step3}")
        return step1, step2, step3, 0, 0 # returning 0's for step4 and step5 so return matches Zilin's IK implementation, makes changing implementations simpler
    
    def getCoords(self):
        return self.x, self.y, self.z

    def updateCoords(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    

if __name__ == "__main__":
    IK = my_InverseKinematics()
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
    dx, dy, dz = 0, 0, -7
    while dz < 7:
                print(f"dx: {dx}, dy: {dy}, dz: {dz}")
                IK.FindStep(dx,dy,dz,0,0)
                dz += 1
