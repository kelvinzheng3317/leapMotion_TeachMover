import threading
import time
import signal

stop_condition = False
# Create an Event object for stopping the main program
stop_event = threading.Event()

def worker():
    while not stop_condition:
        print("Thread running")
        time.sleep(1)
    print("Thread stopping")

# Signal handler for SIGINT (Ctrl+C)
def signal_handler(sig, frame):
    print("Caught SIGINT, stopping threads...")
    stop_event.set()  # Signal threads to stop

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

try: 
    thread = threading.Thread(target=worker)
    thread.start()

    while not stop_event.is_set():
        print("main program infinite loop")
        time.sleep(1)
finally:
    print("Main program stopping")
    stop_event.set()

    stop_condition = True
    thread.join()

    print("Cleanup done")
