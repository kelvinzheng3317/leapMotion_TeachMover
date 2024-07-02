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
import signal

from my_teachMover import TeachMover
from IK_Zilin import InverseKinematics
from IK import my_InverseKinematics
from buffer import Buffer

# controls the stopping of the loops within main and the buffer_execution thread
stop_thread = False
stop_event = threading.Event()

# Signal handler for SIGINT (Ctrl+C)
def signal_handler(sig, frame):
    print("Caught SIGINT, stopping threads...")
    stop_event.set()  # Signal threads to stop

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)


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
    if (math.sqrt(abs_diff[0]**2 + abs_diff[1]**2 + abs_diff[2]**2) < 25):
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
    index_base = np.array(list(subtract_vec(hand.digits[1].proximal.next_joint, hand.digits[1].metacarpal.next_joint)))
    index_tip = np.array(list(subtract_vec(hand.digits[1].distal.next_joint, hand.digits[1].intermediate.next_joint)))
    middle_base = np.array(list(subtract_vec(hand.digits[2].proximal.next_joint, hand.digits[2].metacarpal.next_joint)))
    middle__tip = np.array(list(subtract_vec(hand.digits[2].distal.next_joint, hand.digits[2].intermediate.next_joint)))
    
    # print(np.dot(index_base_dir, index_tip_dir)/(np.linalg.norm(index_base_dir) * np.linalg.norm(index_tip_dir)))
    # print(np.dot(middle_base_dir, middle__tip_dir)/(np.linalg.norm(middle_base_dir) * np.linalg.norm(middle__tip_dir)))

    # Checks if the index and middle finger are straightened out
    index_straight = np.dot(index_base, index_tip)/(np.linalg.norm(index_base) * np.linalg.norm(index_tip))
    middle_straight = np.dot(middle_base, middle__tip)/(np.linalg.norm(middle_base) * np.linalg.norm(middle__tip))
    if index_straight < 0.9 or middle_straight < 0.9:
        return False

    # Checks if the angle btw the index and middle finger in respects to the palm plane is large enough
    palm_norm = np.array(list(hand.palm.normal))
    index_ortho = index_tip - np.dot(index_tip, palm_norm) * palm_norm
    middle_ortho = middle__tip - np.dot(middle__tip, palm_norm) * palm_norm
    angle = math.acos(np.dot(index_ortho, middle_ortho)/ (np.linalg.norm(index_ortho) * np.linalg.norm(middle_ortho)))
    if angle > 0.4:
        return True
    else:
        return False

def execute_buffer(robot, buffer, IK):
    while not stop_thread:
        line = buffer.get()
        if line is None:
            # print("No currently pending command")
            time.sleep(1)
        elif line == "Fist":
            robot.returnToStart()
        elif line == "V":
            robot.open_grip()
        elif line == "Pinch":
            robot.close_grip()
        elif isinstance(line, list) and len(line) == 3:
            j1, j2, j3, j4, j5 = IK.FindStep(line[0], line[1], line[2], 0)
            if j1 != 0 or j2 != 0 or j3 !=0 or j4 != 0 or j5 != 0:
                print(f"----- Moving Robot to ({line[0]}, {line[1]}, {line[2]})-----")
                # print(f"target motor steps: {j1} {j2} {j3} {j4} {j5}")
                # IK.incrCoords(diffX, diffY, diffZ)
                robot.set_step(240, j1, j2, j3, j4, j5, robot.m6)
                print("------------------------")
    print("Stopping thread")

def main():
    # Note that this means we will only have tracking events
    tracking_listening = LatestEventListener(leap.EventType.Tracking)
    connection = leap.Connection()
    connection.add_listener(tracking_listening)
    robot = TeachMover('COM3')
    global stop_thread

    try:
        with connection.open() as open_connection:
            wait_until(lambda: tracking_listening.event is not None)
            IK = my_InverseKinematics()
            buffer = Buffer()
            thread = threading.Thread(target=execute_buffer, args=(robot, buffer, IK))
            thread.start()

            firstFrame = True
            prevX, prevY ,prevZ = 0, 0, 0

            while not stop_event.is_set():
                time.sleep(0.5)
                event = tracking_listening.event
                # events are like: <leap.events.TrackingEvent object at 0x0000010B51736E10>
                if event is not None:
                    # finger: distal, intermediate, proximal, metacarpal
                    if len(event.hands) > 0:
                        hand = event.hands[0]
                        
                        x = hand.palm.position[2]  * 0.016 + 7
                        y = hand.palm.position[0] * 0.05
                        z = hand.palm.position[1] * 0.05 - 3
                        print(f"x: {x}, y: {y}, z: {z} ")

                        diffX = x - IK.x
                        diffY = y - IK.y
                        diffZ = z - IK.z

                        # Moves robot based off of inverse kinematics
                        if firstFrame:
                            firstFrame = False
                        else:
                            # print(f"total distance change = {math.sqrt((diffX)**2 + (diffY)**2 + (diffZ)**2)}")
                            if math.sqrt((diffX)**2 + (diffY)**2 + (diffZ)**2) > 0.5:
                                buffer.add([x,y,z])

                        prevX = x
                        prevY = y
                        prevZ = z

                        # Returns to hand to its 0 position if user makes a O with their hands
                        if hand.grab_strength > 0.95:
                            print("Fist position")
                            buffer.add("Fist")
                        elif V_pos(hand):
                            print("V position")
                            buffer.add("V")
                        else:
                            # FIXME: Set isClosed to reflect whether the robot grip is already closed or not
                            # isClosed = True
                            # if (not isClosed):
                            thumb = hand.digits[0].distal.next_joint
                            index = hand.digits[1].distal.next_joint
                            isPinching, diffs =  pinching(thumb, index)
                            if (isPinching):
                                print(f"Pinching, distances are: [{diffs[0]}, {diffs[1]}, {diffs[2]}]")
                                buffer.add("Pinch")
                        
                        '''
                        if not robot.lock.locked():
                            line = buffer.get()
                            if line is None:
                                print("No currently pending command")
                            elif line == "Fist":
                                robot.returnToStart()
                            elif line == "V":
                                robot.open_grip()
                            elif line == "Pinch":
                                robot.close_grip()
                            elif isinstance(line, list) and len(line) == 3:
                                j1, j2, j3, j4, j5 = IK.FindStep(line[0], line[1], line[2], 0)
                                if j1 != 0 or j2 != 0 or j3 !=0 or j4 != 0 or j5 != 0:
                                    print(f"----- Moving Robot to ({line[0]}, {line[1]}, {line[2]})-----")
                                    # print(f"target motor steps: {j1} {j2} {j3} {j4} {j5}")
                                    # IK.incrCoords(diffX, diffY, diffZ)
                                    robot.set_step(240, j1, j2, j3, j4, j5, robot.m6)
                                    print("------------------------")
                        '''

                    # case where no tracking event is occuring
                    else:
                        firstFrame = True
                        print("No hand detected")
    finally:
        print("Stopping main program")
        stop_thread = True
        thread.join()
        print("Finish program cleanup")
        


if __name__ == "__main__":
    main()