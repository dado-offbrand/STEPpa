import numpy as np
import gait
import ik

class Quadruped:
    _GROUPS = ["Front Left", "Front Right", "Back Left", "Back Right"]

    # Initializes a quadruped at neutral position
    def __init__(self):
        self.back_left = Leg("Back Left")
        self.front_left = Leg("Front Left")
        self.front_right = Leg("Front Right")
        self.back_right = Leg("Back Right")

        self.legs = [self.front_left, self.front_right, self.back_left, self.back_right]
        self.gait_planner = gait.gait(self)

        self.movement_vector = 1

        self.SIM_LOCO_GUI = 2
        self.SIM_LOCO_IK = 1
        self.sim_loco_type = self.SIM_LOCO_IK

    # Updates all leg poses based off of an array of targets
    def update_all_poses(self, targets):
        for i, (tx, ty, tz) in enumerate(targets):
            try:
                self.legs[i].set_foot_target(tx, ty, tz)
                self.legs[i].update_pose()
            except ValueError:
                print("[DEBUG] Requested angle is outside of range of motion")
                pass # pose is out of range, do not update (temp solution, implement flooring)

    # Get the foot target tuples in a (4, 3) array in order of _GROUPS
    def get_target_tuples(self):
        return [leg.get_foot_target() for leg in self.legs]
    
    # Return the flat pose array of the entire quadruped in order for isaacsim
    def get_pose_array(self):
        poses = [leg.get_pose() for leg in self.legs]
        return np.array([p[0] for p in poses] +   # shoulders
                    [p[1] for p in poses] +   # uppers
                    [p[2] for p in poses])     # lowers

class Leg:
    # Initializes a leg at neutral position
    def __init__(self, group):
        # Assign actuator thetas (shoulder, upper, lower)
        self._pose = np.array([
            0, ik._UN, ik._LN
        ])

        # Assign targets based off of thetas
        self.targ_x, self.targ_y, self.targ_z = ik.leg_fk(self._pose[0], self._pose[1], self._pose[2])
        # Assign group name
        self.group = group

    # Sets foot target position
    def set_foot_target(self, x, y, z):
        self.targ_x = x
        self.targ_y = y
        self.targ_z = z

    # Gets the foot target as a tuple
    def get_foot_target(self):
        return (self.targ_x, self.targ_y, self.targ_z)

    # Updates pose based off of target position
    def update_pose(self):
        self._pose[0], self._pose[1], self._pose[2] = ik.leg_ik(self.targ_x, self.targ_y, self.targ_z)

    # Get (a copy of) current pose
    def get_pose(self):
        return self._pose.copy()