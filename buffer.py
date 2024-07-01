class Buffer:
    def __init__(self):
        self.data = []
    
    def add(self, msg):
        if len(self.data) > 4:
            print("Buffer is full")
        else:
            self.data.append(msg)

    def get(self):
        if len(self.data) > 0:
            msg = self.data.pop()
            return msg
        else:
            print("No data in buffer")