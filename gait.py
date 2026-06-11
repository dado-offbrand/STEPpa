import trajectory

class gait:
    def __init__(self):
        self.back_left_tp = trajectory.CycloidTrajectory(0, 0, 0)
        self.front_right_tp = trajectory.CycloidTrajectory(0, 0, -0.02)
        self.back_right_tp = trajectory.CycloidTrajectory(0.5, 0, 0)
        self.front_left_tp = trajectory.CycloidTrajectory(0.5, 0, -0.02)
        self.tps = [self.back_left_tp, self.front_left_tp, self.front_right_tp, self.back_right_tp]

        self.forward_vector = 1
        #self.rotational_vector = 1 # how much further the left/right extends

    def cycle(self):
        targets = []
        for tp in self.tps:
            x, y, z = tp.cycle()
            targets.append((x * self.movement_vector, y, z))
        self.update_all_poses(targets)

# moving can be done by setting individual movement vectors on feet to rotate, along with
# forward offset so that one foot moves more than the other for turning while walking?
# an actual ik model of the robot from example cara code may be a necessary guide or something
# there is no positional lerping
# try to remake everything from IK based off of aead's code by just modifying it and converting it to python and replacing the
# motor movement logic with isaac sim movement logic. straighten out which joint is which while this is happening