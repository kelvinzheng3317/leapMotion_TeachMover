class Buffer:
    def __init__(self):
        self.data = []
        self.hasContent = False
    
    def add(self, msg):
        if len(self.data) > 4:
            print("Buffer is full")
        else:
            self.data.append(msg)
            self.hasContent = True

    def get(self):
        if self.hasContent:
            msg = self.data.pop()
            return msg
        else:
            print("No data in buffer")