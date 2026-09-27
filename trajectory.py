import math

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

if __name__=='__main__':
    t = CycloidTrajectory()
    t.debug_dump()