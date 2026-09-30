import math
import numpy as np

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
        if t < self.duty_cycle:
            u = t / self.duty_cycle
            x = x2 + (x1 - x2) * u
            y = y2 + (y1 - y2) * u
            z = z2 + (z1 - z2) * u
        else:
            # swing phase (cycloidal step)
            u = (t - self.duty_cycle) / (1 - self.duty_cycle)
            x = x1 + (x2 - x1) * u
            y = y1 + (y2 - y1) * u

            A = self.STEP_HEIGHT

            z = (
                (1 - u) * z1
                + u * z2
                + A * (1 - np.cos(2 * np.pi * u)) / 2
            )

        self.phase += self.dt
        return (x, y, z)

    def debug_dump(self, steps=200):
        step_points = []
        
        for _ in range(steps):
            point = self.cycle2(-0.09, 0.09, -0.045435, 0.07, -0.22, -0.22)
            print(point)
            step_points.append(f'({point[0]}, {point[1]}, {str(point[2])})')

        with open ('points.txt', 'w') as f:
            for point in step_points:
                f.write(f'{point}\n')

if __name__=='__main__':
    t = CycloidTrajectory3D(0.6, 0.0, -0.01)
    t.debug_dump()