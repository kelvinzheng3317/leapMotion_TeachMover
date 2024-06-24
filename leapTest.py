import leap
import time
import math
import numpy as np
from timeit import default_timer as timer
from typing import Callable
import threading
from leap.events import TrackingEvent
from leap.event_listener import LatestEventListener
import leap.datatypes as ldt

from my_teachMover import TeachMover
from IK_Zilin import InverseKinematics


def wait_until(condition: Callable[[], bool], timeout: float = 5, poll_delay: float = 0.01):
    start_time = timer()
    while timer() - start_time < timeout:
        if condition():
            return True
        time.sleep(poll_delay)
    if not condition():
        return False
    

# this only works on values of type leap.datatypes.Vector 
def subtract_vec(finger1: ldt.Vector, finger2: ldt.Vector):
    return map(float.__sub__, finger1, finger2)

def find_distance(point1: ldt.Vector, point2: ldt.Vector):
    return list(map(abs, subtract_vec(point1, point2)))

def pinching(finger1: ldt.Vector, finger2: ldt.Vector):
    abs_diff = find_distance(finger1, finger2)
    # print(f"thumb and index distance: {math.sqrt(abs_diff[0]**2 + abs_diff[1]**2 + abs_diff[2]**2)}")
    if (math.sqrt(abs_diff[0]**2 + abs_diff[1]**2 + abs_diff[2]**2) < 20):
        return True, abs_diff
    else:
        return False, abs_diff

# FIXME: probably need to rework this function as its often being triggered by the pinching gesture
def O_pos(hand: ldt.Hand):
    thumb = hand.digits[0].distal.next_joint
    index = hand.digits[1].distal.next_joint
    middle = hand.digits[2].distal.next_joint
    ring = hand.digits[3].distal.next_joint
    pinky = hand.digits[4].distal.next_joint
    if (pinching(thumb, index)[0] and pinching(thumb, middle)[0] and pinching(thumb, ring)[0] and pinching(thumb, pinky)[0]):
        return True
    else:
        return False

# aims to check if there is a large angle btw index and middle finger on the plane that is orthogonal to the palm normal
def V_pos(hand):
    index_dir = np.array(list(subtract_vec(hand.digits[1].distal.next_joint, hand.digits[1].metacarpal.next_joint)))
    middle_dir = np.array(list(subtract_vec(hand.digits[2].distal.next_joint, hand.digits[2].metacarpal.next_joint)))
    palm_norm = np.array(list(hand.palm.normal))
    index_ortho = index_dir - np.dot(index_dir, palm_norm) * palm_norm
    middle_ortho = middle_dir - np.dot(middle_dir, palm_norm) * palm_norm
    angle = math.acos(np.dot(index_ortho, middle_ortho)/ (np.linalg.norm(index_ortho) * np.linalg.norm(middle_ortho)))
    if angle > 0.4:
        return True
    else:
        return False

def main():
    # Note that this means we will only have tracking events
    tracking_listening = LatestEventListener(leap.EventType.Tracking)

    connection = leap.Connection()
    connection.add_listener(tracking_listening)

    robot = TeachMover('COM3')

    with connection.open() as open_connection:
        wait_until(lambda: tracking_listening.event is not None)
        IK = InverseKinematics(0,0,0,0,0)
        firstFrame = True
        prevX, prevY ,prevZ = 0, 0, 0
        while True:
            time.sleep(0.5)
            event = tracking_listening.event
            # events are like: <leap.events.TrackingEvent object at 0x0000010B51736E10>
            if event is not None:
                # finger: distal, intermediate, proximal, metacarpal
                if len(event.hands) > 0:
                    hand = event.hands[0]
                    x = hand.palm.position[2] * 0.1
                    y = hand.palm.position[0] * 0.1
                    z = hand.palm.position[1] * 0.1
                    print(f"x: {x}, y: {y}, z: {y} ")
                    
                    # Moves robot based off of inverse kinematics
                    if firstFrame:
                        firstFrame = False
                    else:
                        j1, j2, j3, j4, j5 = IK.FindStep(x - prevX, y - prevY, z - prevZ, 0, 0)
                        robot.set_step(240, j1, j2, j3, j4, j5, 0)

                    # NOTE: MASSIVE PAUSE IN PROGRAM EVERYTIME THERE IS A ROBOT MOVE -> PROBABLY NEED TO IMPLEMENT THREADS
                    # xChange, yChange, zChange = 0, 0, 0
                    # if (x - prevX > 50):
                    #     print("moving right")
                    #     xChange = 100
                    # elif (x - prevX < -50):
                    #     print("moving left")
                    #     xChange = -100

                    # if (y - prevY > 50):
                    #     print("moving up")
                    #     yChange = -100
                    # elif (y - prevY < -50):
                    #     print("moving down")
                    #     yChange = 100
                    
                    # if (z - prevZ > 50):
                    #     print("moving forward")
                    #     zChange = 100
                    # elif (z - prevZ < -50):
                    #     print("moving backwards")
                    #     zChange = -100

                    # if (xChange != 0 or yChange != 0 or zChange != 0):
                    #     robot.move(200, xChange, yChange, zChange, 0, 0, 0)

                    prevX = x
                    prevY = y
                    prevZ = z

                    # Returns to hand to its 0 position if user makes a O with their hands
                    if O_pos(hand):
                        print("Hand in O position: ", end="")
                        robot.returnToZero()
                    elif V_pos(hand):
                        print("Hand in V position: ", end="")
                        robot.open_grip()
                    else:
                        # FIXME: Set isClosed to reflect whether the robot grip is already closed or not
                        # isClosed = True
                        # if (not isClosed):
                        thumb = hand.digits[0].distal.next_joint
                        index = hand.digits[1].distal.next_joint
                        isPinching, diffs =  pinching(thumb, index)
                        if (isPinching):
                            print(f"Pinching, distances are: [{diffs[0]}, {diffs[1]}, {diffs[2]}]")
                            robot.close_grip()
                
                # case where no tracking event is occuring
                else:
                    firstFrame = True
                    print("No hand detected")


if __name__ == "__main__":
    main()