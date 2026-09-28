# -----------------------------
# Main Imports
# -----------------------------
from isaacsim import SimulationApp
import keyboard
import numpy as np
import time
import math
import ik
import sim_ui
import quadruped
import trajectory

# Keep this before anything else for other imports to work
simulation_app = SimulationApp({"headless": False})

# -----------------------------
# Simulation Imports (only available at runtime)
# -----------------------------
import omni.usd
import omni.physx
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Sdf, UsdLux, UsdPhysics

# -----------------------------
# Create world
# -----------------------------
world = World(stage_units_in_meters=1.0)

# Lighting
stage = omni.usd.get_context().get_stage()
dome = UsdLux.DomeLight.Define(stage, "/World/Light/DomeLight")
dome.CreateIntensityAttr(1000)

# -----------------------------
# Create quadruped
# -----------------------------

# Quadruped reference
add_reference_to_stage(
    usd_path="quadrupedX.usd",
    prim_path="/World/ROS2"
)

# -----------------------------
# Begin world simulation
# -----------------------------

# Jank fix to wait for everything to load
world.play()
world.reset()

# -----------------------------
# Robot Setup
# -----------------------------
robot = Articulation(prim_paths_expr="/World/ROS2")
robot.initialize()

quad = quadruped.Quadruped()
gui = sim_ui.TargetUI(quad.get_target_tuples())

# -----------------------------
# Main loop
# -----------------------------

while simulation_app.is_running():
    gui.update()

    # Make GUI reflect current quadruped targets
    if (gui.viewonly):
        print("viewonly behavior has not been finished")
        #gui.set_targets(quad.get_target_tuples())
        # set quadruped tuples another way here ig?
    # Set quadruped pose from targets
    else:
        #---------------------------
        # old code for using gui positions
        #---------------------------

        #targets = gui.get_targets()
        #quad.update_all_poses(targets)

        #---------------------------
        # new code for trot test
        #---------------------------

        # quadruped targets should be in form [{x, y, z}, ...] per leg as tuples?
        #targets=list, list items=triple tuple

        # pose limits: x (forward): -0.221, 0.221
        #              y (lateral): -0.225, 0.225
        #             z (vertical): -0.300, -0.10

        if keyboard.is_pressed('tab'):
            quad.gait_planner.cycle()
        elif keyboard.is_pressed('shift'):
            quad.gait_planner.forward_vector *= -1

    # Send pose to robot
    robot.set_joint_position_targets(quad.get_pose_array())
    
    # Step physics simulation
    world.step(render=True)

gui.close()
simulation_app.close()