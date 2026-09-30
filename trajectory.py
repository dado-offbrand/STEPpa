import math
import numpy as np

class CycloidTrajectory:
    def __init__(self, phase_offset, forward_offset, height_offset):
        self.START_X = -0.09
        self.END_X = 0.09 + forward_offset # this will lead to a bug where you cannot turn moving backwards
        self.Z_BASE = -0.22 + height_offset # default of 0 for back legs
        self.STEP_HEIGHT = 0.025

        self.dt = 0.01
        self.phase = 0.0 + phase_offset

        self.duty_cycle = 0.6  # stance ratio

    def set_offsets(phase_offset, forward_offset, height_offset):
        self.phase = 0.0 + phase_offset
        self.END_X = 0.0 + forward_offset
        self.Z_BASE = -0.22 + height_offset

    def cycle(self):
        t = self.phase % 1.0

        if t < self.duty_cycle:
            # STANCE PHASE
            u = t / self.duty_cycle

            x = self.END_X - (self.END_X - self.START_X) * u
            z = self.Z_BASE

        else:
            # SWING PHASE
            u = (t - self.duty_cycle) / (1 - self.duty_cycle)

            x = self.START_X + (self.END_X - self.START_X) * u

            # Cycloid (smooth lift)
            z = self.Z_BASE + self.STEP_HEIGHT * (1 - math.cos(2 * math.pi * u)) / 2

        self.phase += self.dt
        return x, 0.045435, z  # y fixed temporarily

    def debug_dump(self, steps=200):
        for _ in range(steps):
            print(self.cycle())

class CycloidTrajectory3D:
    def __init__(self, phase_offset, forward_offset, height_offset):
        self.START_X = -0.09
        self.END_X = 0.09 + forward_offset # this will lead to a bug where you cannot turn moving backwards
        
        self.START_Y = 0
        self.END_Y = 0
        
        self.Z_BASE = -0.22 + height_offset # default of 0 for back legs
        self.STEP_HEIGHT = 0.03 # 0.015 # 0.025

        self.dt = 0.01
        self.phase = 0.0 + phase_offset

        self.duty_cycle = 0.6 # decrease to spend more time on ground?

    def set_offsets(self, phase_offset, forward_offset, height_offset):
        self.phase = 0.0 + phase_offset
        self.END_X = 0.0 + forward_offset
        self.Z_BASE = -0.22 + height_offset

    def cycle2(self, x1, x2, y1, y2, z1, z2):
        t = self.phase % 1.0

        # swing phase (straight line)
        #if t < self.duty_cycle:
        if t > self.duty_cycle:
            u = t / self.duty_cycle
            x = x1 + (x2 - x1) * u
            y = y1 + (y2 - y1) * u
            z = z1 + (z2 - z1) * u
            #print('straight')
        else:
            # swing phase (cycloidal step)
            u = (t - self.duty_cycle) / (1 - self.duty_cycle)
            x = x1 + (x2 - x1) * u
            y = y1 + (y2 - y1) * u

            #A = self.STEP_HEIGHT - 0.5 * (z1 + z2) # this code does not work
            A = self.STEP_HEIGHT

            z = (
                (1 - u) * z1
                + u * z2
                + A * (1 - np.cos(2 * np.pi * u)) / 2
            )
            #print('swing')

        self.phase += self.dt
        return (x, y, z)

    def cycle(self):
        t = self.phase % 1.0

        if t < self.duty_cycle:
            # STANCE PHASE
            u = t / self.duty_cycle

            x = self.END_X - (self.END_X - self.START_X) * u
            z = self.Z_BASE

        else:
            # SWING PHASE
            u = (t - self.duty_cycle) / (1 - self.duty_cycle)

            x = self.START_X + (self.END_X - self.START_X) * u

            # Cycloid (smooth lift)
            z = self.Z_BASE + self.STEP_HEIGHT * (1 - math.cos(2 * math.pi * u)) / 2

        self.phase += self.dt
        return x, 0.045435, z  # y fixed temporarily

    def debug_dump(self, steps=200):
        step_points = []
        
        for _ in range(steps):
            point = self.cycle2(-0.09, 0.09, -0.045435, 0.07, -0.22, -0.22)
            print(point)
            step_points.append(f'({point[0]}, {point[1]}, {str(point[2])})')
            #input()
            #print(self.cycle())

        with open ('points.txt', 'w') as f:
            for point in step_points:
                f.write(f'{point}\n')

if __name__=='__main__':
    t = CycloidTrajectory3D(0.0, 0.0, 0.0)
    t.debug_dump()