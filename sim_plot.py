from mpl_toolkits import mplot3d
import matplotlib.animation as animation
import numpy as np
from math import *
import matplotlib.pyplot as plt
import trajectory
import quadruped
import keyboard

def init_axis(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax



def main():
    quad = quadruped.Quadruped()
    tp = trajectory.CycloidTrajectory3D(0, 0, 0.01)

    leg = 0
    ctargs = quad.gait_planner.get_y_ctargs()
    x = []
    y = []
    z = []

    for i in range(100):
        cx, cy, cz = tp.cycle2(ctargs[leg][0], ctargs[leg][1], ctargs[leg][2], ctargs[leg][3], tp.Z_BASE, tp.Z_BASE)
        x.append(cx)
        y.append(cy)
        z.append(cz)

    figure = plt.figure()
    ax = init_axis(.1)

    ax.plot3D(x, y, z, 'ro')
    plt.show()

if __name__=='__main__':
    main()