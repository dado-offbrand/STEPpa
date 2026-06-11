# -----------------------------
# Main Imports
# -----------------------------
import numpy as np
import time
import math

# -----------------------------
# Leg IK CONSTANTS
# -----------------------------
LOWER_LEG_LENGTH = 0.180 # constant because lower leg length was accidentally left too long (subtract foot length!)

_D  = 0.045435         # shoulder lateral offset
_L1 = 0.134            # upper leg pulley distance
_L2 = LOWER_LEG_LENGTH # pulley to center of foot distance

_UN = np.deg2rad(-45) # upper neutral angle
_LN = np.deg2rad(-90) # lower neutral angle


def leg_ik(x, y, z, d=_D, l1=_L1, l2=_L2):
    """
    Input: (x, y, z) foot position in the shoulder joint's local frame (metres).
           +x  forward,  +y  outward (positive for left, negative for right),  +z  up
    Output: (theta_shoulder, theta_upper, theta_lower) in radians.
    """
    # Shoulder (abduction / adduction)
    r_yz = math.hypot(y, z)
    if r_yz < d:
        raise ValueError(f"Target too close to body axis: r_yz={r_yz:.4f} < d={d:.4f}")
    theta_shoulder = math.atan2(y, -z) - math.atan2(d, math.sqrt(r_yz**2 - d**2))

    # Project into sagittal plane
    L_vert = math.sqrt(max(r_yz**2 - d**2, 0.0))
    R = math.hypot(x, L_vert)
    if R > l1 + l2:
        raise ValueError(f"Target unreachable: R={R:.4f} > max reach {l1+l2:.4f}")
    if R < abs(l1 - l2):
        raise ValueError(f"Target unreachable: R={R:.4f} < min reach {abs(l1-l2):.4f}")

    # Knee angle (law of cosines); negate for the -X lower-joint axis
    cos_knee = max(-1.0, min(1.0, (R**2 - l1**2 - l2**2) / (2.0 * l1 * l2)))
    theta_lower = -math.acos(cos_knee)

    # Upper angle
    theta_knee = -theta_lower
    theta_upper = math.atan2(x, L_vert) - math.atan2(l2 * math.sin(theta_knee),
                                                       l1 + l2 * math.cos(theta_knee))
    return theta_shoulder, theta_upper, theta_lower


def leg_fk(theta_shoulder, theta_upper, theta_lower, d=_D, l1=_L1, l2=_L2):
    """
    Forward kinematics function, returns (x, y, z) foot position.
    """
    theta_lower_global = theta_upper - theta_lower          # accounts for -X axis
    x    = l1 * math.sin(theta_upper) + l2 * math.sin(theta_lower_global)
    down = l1 * math.cos(theta_upper) + l2 * math.cos(theta_lower_global)
    r    = math.hypot(d, down)
    total_angle = math.atan2(d, down) + theta_shoulder
    y =  r * math.sin(total_angle)
    z = -r * math.cos(total_angle)
    return x, y, z

def get_pose_neutral():
    """
    Returns angles for neutral pose
    Joint order: Back Left, Front Left, Front Right, Back Right (per group)
    Indices:     0          1            2             3           (shoulders)
                 4          5            6             7           (uppers)
                 8          9            10            11          (lowers)
    """

    return np.array([
        0, 0, 0, 0,
        _UN, _UN, _UN, _UN,
        _LN, _LN, _LN, _LN,
    ])