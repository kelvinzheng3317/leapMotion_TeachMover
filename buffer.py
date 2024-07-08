import threading

class Buffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()
    
    def add(self, msg):
        with self.lock:
            self.data = msg

    def get(self):
        with self.lock:
            msg = self.data
            self.data = None
            return msg
        
    def isEmpty(self):
        return self.data is None


if __name__ == "__main__":
    buffer = Buffer()
    print(buffer.get())

    buffer.add(1)
    buffer.add("Fist")
    buffer.add([1,2,3])
    item = buffer.get()
    print(item)