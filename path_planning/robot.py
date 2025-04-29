import math

class Robot:
    def __init__(self):
        self.direction = 0 # Direction of robot's movement 
        self.speed = 0 # Speed of robot's movement  
        self.rotation = 0 # Rotation of robot during movement
        self.turn_speed = 100 # Speed limit of the rotation

        self.angle = 0 # Current robot rotation (from gyroscope)
        self.coord = [0, 0] # Position of the robot on field

        self.motor_on = True
        self.dribbling_on = False
        self.kicker_on = False
        self.force_kick = False
        self.angle_offset = 0

        self.running = True
    
    def calc_coord(self, angle, dist):
        self.coord[0] = -math.sin((angle + self.angle) / 180 * math.pi) * dist
        self.coord[1] = -math.cos((angle + self.angle) / 180 * math.pi) * dist
        