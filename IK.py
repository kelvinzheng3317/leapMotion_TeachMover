import math

class InverseKinematics:
    C = 2 * math.pi     # Radians of a circle
    B_C = 7072 / C      # Base motor:             7072 steps in 1 rotation
    S_C = 7072 / C      # Shoulder motor:         7072 steps in 1 rotation
    E_C = 4158 / C      # Elbow motor:            4158 steps in 1 rotation
    W_C = 1536 / C      # Right/Left Wrist motor: 1536 steps in 1 rotation
    G_C = 2330 / C      # Gripper motor:          2330 steps in 1 rotation

    # TeachMover2 Geometrical Paramaters (inches)
    H = 7.625   # The length of Base to Shoulder
    L = 7.0     # The length of Shoulder to Elbow and Elbow to Wrist
    LL = 3.8    # The length of Wrist

    def __init__(self):
        return
    
    def FindStep(self):
        return