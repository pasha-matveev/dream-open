class Robot:
    def __init__(self):
        self.angle = 0
        self.speed = 0
        self.direction = 0
        self.turn_angle = 0
        self.turn_speed = 100

        self.coord = [0, 0]

        self.motor_on = False
        self.dribbling_on = False
        self.kicker_on = False
        self.force_kick = False
        self.angle_offset = 0

        self.running = True