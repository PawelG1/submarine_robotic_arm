#servo helper class
class Servo:

    def __init__(self, id: int, value: int = 90):
        self.id = id
        self.value = value

    def setValue(self, value: int):
        self.value = value
