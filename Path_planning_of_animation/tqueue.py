"""

"""


class Queue(object):
    def __init__(self):
        self.queue = []

    def put(self, index):
        self.queue.append(index)

    def get(self):
        return self.queue.pop(0)

    def empty(self):
        return True if len(self.queue) == 0 else False