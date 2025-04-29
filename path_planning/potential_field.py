from models import *

class PotentialField:
    def __init__(self, obj: Circle):
        self.obj = obj
    
    def attract(self, robot: Robot):
        pass
    
    def detract(self, robot: Robot):
        pass
    
    def restrict(self, robot: Robot):
        pass