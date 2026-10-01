import trajectory

class gait:
    def __init__(self, quad):
        self.back_left_tp = trajectory.CycloidTrajectory3D(0, 0, 0.01)
        self.front_right_tp = trajectory.CycloidTrajectory3D(0, 0, 0.005)
        self.back_right_tp = trajectory.CycloidTrajectory3D(0.5, 0, 0.01)
        self.front_left_tp = trajectory.CycloidTrajectory3D(0.5, 0, 0.005)
        self.tps = [
            self.front_left_tp, 
            self.front_right_tp, 
            self.back_left_tp, 
            self.back_right_tp
        ]

        self.forward_vector = 0
        self.side_vector = 0

        self.quad = quad

    # Cycle targets for x movement only
    def get_x_ctargs(self):
        return [
            [-0.03, 0.03, 0.045435, 0.045435], # front left
            [-0.03, 0.03, 0.045435, 0.045435], # front right
            [-0.03, 0.03, 0.045435, 0.045435], # back left
            [-0.03, 0.03, 0.045435, 0.045435] # back right
        ]

    # Cycle targets for y movement only
    def get_y_ctargs(self):
        return [
            [0.0, 0.0, 0.045435, 0.045435], # front left
            [0.0, 0.0, 0.045435, 0.045435], # front right
            [0.0, 0.0, 0.045435, 0.045435], # back left
            [0.0, 0.0, 0.045435, 0.045435] # back right
        ]

    # Cycle targets for zero movement but still trotting
    def get_no_ctargs(self):
        return [
            [0.0, 0.0, 0.045435, 0.045435], # front left
            [0.0, 0.0, 0.045435, 0.045435], # front right
            [0.0, 0.0, 0.045435, 0.045435], # back left
            [0.0, 0.0, 0.045435, 0.045435] # back right
        ]

    # Returns the targets after one gait cycle
    def cycle(self):
        targets = []

        i = 0
        for tp in self.tps:
            print(self.forward_vector)
            print(self.side_vector)
            if (not self.forward_vector==0 and self.side_vector==0):
                #print('opt1')
                ctargs = self.get_x_ctargs()
            elif (self.forward_vector==0 and not self.side_vector == 0):
                #print('opt2')
                ctargs = self.get_y_ctargs()
            elif (self.forward_vector == 0 and self.side_vector == 0):
                #print('opt3')
                ctargs = self.get_x_ctargs()
            else:
                print('opt4 not done yet')
                continue
            
            x, y, z = tp.cycle2(ctargs[i][0], ctargs[i][1], ctargs[i][2], ctargs[i][3], tp.Z_BASE, tp.Z_BASE)
            # if (not self.side_vector == 0):
            #     targets.append((x * self.forward_vector, y if i==3 or i==4 else y*self.side_vector, z))
            # else:
            #     targets.append((x * self.forward_vector, y, z * self.side))
            targets.append((x * self.forward_vector, y, z * self.side_vector))
            i += 1

        return targets