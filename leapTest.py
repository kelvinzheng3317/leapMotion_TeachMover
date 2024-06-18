import leap
import time
from timeit import default_timer as timer
from typing import Callable
from leap.events import TrackingEvent
from leap.event_listener import LatestEventListener
import leap.datatypes as ldt

from my_teachMover import TeachMover


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


def pinching(finger1: ldt.Vector, finger2: ldt.Vector):
    abs_diff = list(map(abs, subtract_vec(finger1, finger2)))
    if (abs_diff[0] < 20 < 20 or abs_diff[2] < 20):
        return True, abs_diff
    else:
        return False, abs_diff


def main():
    # Note that this means we will only have tracking events
    tracking_listening = LatestEventListener(leap.EventType.Tracking)

    connection = leap.Connection()
    connection.add_listener(tracking_listening)

    robot = TeachMover('COM3')

    with connection.open() as open_connection:
        wait_until(lambda: tracking_listening.event is not None)
        prevX, prevY ,prevZ = 0, 0, 0
        while True:
            time.sleep(0.5)
            event = tracking_listening.event
            # events are like: <leap.events.TrackingEvent object at 0x0000010B51736E10>
            if event is not None:
                # finger: distal, intermediate, proximal, metacarpal
                if len(event.hands) > 0:
                    hand = event.hands[0]
                    x = hand.palm.position.x
                    y = hand.palm.position.y
                    z = hand.palm.position.z
                    print(f"x: {x}, y: {y}, z: {y} ")

                    # NOTE: MASSIVE PAUSE IN PROGRAM EVERYTIME THERE IS A ROBOT MOVE -> PROBABLY NEED TO IMPLEMENT THREADS
                    xChange, yChange, zChange = 0, 0, 0
                    if (x - prevX > 50):
                        print("moving right")
                        xChange = 100
                    elif (x - prevX < -50):
                        print("moving left")
                        xChange = -100

                    if (y - prevY > 50):
                        print("moving up")
                        yChange = -100
                    elif (y - prevY < -50):
                        print("moving down")
                        yChange = 100
                    
                    if (z - prevZ > 50):
                        print("moving forward")
                        zChange = 100
                    elif (z - prevZ < -50):
                        print("moving backwards")
                        zChange = -100

                    if (xChange != 0 or yChange != 0):
                        robot.move(200, xChange, yChange, zChange, 0, 0, 0)
                    
                    prevX = x
                    prevY = y
                    prevZ = z

                    thumb = hand.digits[0].distal.next_joint
                    index = hand.digits[1].distal.next_joint
                    isPinching, diffs =  pinching(thumb, index)
                    if (isPinching):
                        print(f"Pinching, distances are: [{diffs[0]}, {diffs[1]}, {diffs[2]}]")
                        robot.close_grip()
                
                # case where no tracking event is occuring
                else:
                    print("No hand detected")


if __name__ == "__main__":
    main()