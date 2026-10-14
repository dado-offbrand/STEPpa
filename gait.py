import trajectory

class gait:
    def __init__(self, quad):
        self.back_left_tp = trajectory.CycloidTrajectory3D(0, 0, 0.01)
        self.front_right_tp = trajectory.CycloidTrajectory3D(0, 0, 0.01)
        self.back_right_tp = trajectory.CycloidTrajectory3D(0.5, 0, 0.01)
        self.front_left_tp = trajectory.CycloidTrajectory3D(0.5, 0, 0.01)
        self.tps = [
            self.front_left_tp, 
            self.front_right_tp, 
            self.back_left_tp, 
            self.back_right_tp
        ]

        self.forward_vector = 0
        self.side_vector = 0
        self.rotational_vector = 0
        self.stride = 0.03
        self.rotational_stride = 0.03

        self.quad = quad

    def get_xy_ctargs(self):
        # current stride
        cstride = 0 if self.forward_vector==0 and self.side_vector==0 else self.stride

        # directly under shoulder
        base_y = 0.045435
        base_x = 0.0

        # LEFT/RIGHT MOVEMENT   
        # NOTE logically, stride of 0 results in no y movement     
        stride_y = cstride * self.side_vector 
        
        # left legs
        left_start = base_y - stride_y
        left_end = base_y + stride_y
        
        # right legs
        right_start = base_y + stride_y
        right_end = base_y - stride_y

        # FORWARD/BACkWARD MOVEMENT
        stride_x = cstride * self.forward_vector

        front_start = base_x - stride_x
        front_end_left = base_x + stride_x
        front_end_right = base_x + stride_x

        if (self.rotational_vector == -1):
            front_end_left += (self.rotational_stride)
            front_end_right -= (self.rotational_stride)
        elif (self.rotational_vector == 1):
            front_end_left -= (self.rotational_stride)
            front_end_right += (self.rotational_stride)

        return [
            [front_start, front_end_left, left_start, left_end], # front left
            [front_start, front_end_right, right_start, right_end], # front right
            [front_start, front_end_left, right_start, right_end], # back left
            [front_start, front_end_right, left_start, left_end] # back right
        ]

    # Returns the targets after one gait cycle
    def cycle(self):
        targets = []

        i = 0
        for tp in self.tps:
            ctargs = self.get_xy_ctargs()
            x, y, z = tp.cycle2(ctargs[i][0], ctargs[i][1], ctargs[i][2], ctargs[i][3], tp.Z_BASE, tp.Z_BASE)
            targets.append((x, y, z))
            i += 1

        return targets