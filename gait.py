import trajectory

class gait:
    def __init__(self, quad):
        self.back_left_tp = trajectory.CycloidTrajectory3D(0, 0, -0.6)
        self.front_right_tp = trajectory.CycloidTrajectory3D(0, 0, -0.6)
        self.back_right_tp = trajectory.CycloidTrajectory3D(0.5, 0, -1)
        self.front_left_tp = trajectory.CycloidTrajectory3D(0.5, 0, -1)
        self.tps = [
            self.front_left_tp, 
            self.front_right_tp, 
            self.back_left_tp, 
            self.back_right_tp
        ]

        self.FORWARD = 1
        self.BACKWARD = -1
        self.RIGHT = 2
        self.LEFT = -2

        self.forward_vector = 1
        #self.rotational_vector = 1 # how much further the left/right extends

        self.quad = quad

    def set_vector(self, direction):
        match direction:
            case 1:
                self.forward_vector = 1
                self.side_vector = 0
            case -1:
                self.forward_vector = -1
                self.side_vector = 0
            case 2:
                self.forward_vector = 0
                self.side_vector = 1
            case -2:
                self.forward_vector = 0
                self.side_vector = -1

    # Returns the targets after one gait cycle
    def cycle(self):
        targets = []
        for tp in self.tps:
            
            x, y, z = tp.cycle2(-0.03, 0.03, 0.045435, 0.045435, -0.22, -0.22) # new gait sys actual walking point janky walking point
            targets.append((x * self.forward_vector, y, z))
        return targets