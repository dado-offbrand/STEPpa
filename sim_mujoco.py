import time
import mujoco
import mujoco.viewer
import quadruped
import numpy as np

# URDF loading
model = mujoco.MjModel.from_xml_path('quadruped_mujoco.xml')
data = mujoco.MjData(model)

# Time guff
CONTROL_INTERVAL_MS = 10.0 # run every x miliseconds
control_dt = CONTROL_INTERVAL_MS / 1000.0  # convert to seconds

# Custom class guff
quad = quadruped.Quadruped()
#gui = sim_ui.TargetUI(quad.get_target_tuples())

    # TODO:
    # Init pose from quadruped's array, not this manual stuff.
    # Continue to adjust kp/kv. Limbs are too slow to ease into final position
    # Why does it seem like joint positions affect other joint positions when moving?
    # Test cycloidal trajectory on one leg ig

    # NOTE:
    # All limbs are now controllable

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation running. Close the viewer window to exit.")
    last_control_time = 0.0

    while viewer.is_running():
        if (data.time - last_control_time) >= control_dt:
            #
            # Get quadruped position array
            #
            quad.gait_planner.cycle()
            positions = quad.get_pose_array()

            #
            # Set simulation data
            #
            # Shoulders
            data.ctrl[0] = positions[0]
            data.ctrl[3] = positions[1]
            data.ctrl[6] = positions[2]
            data.ctrl[9] = positions[3]

            # Upper legs
            data.ctrl[1] = positions[4]
            data.ctrl[4] = positions[5]
            data.ctrl[7] = positions[6]
            data.ctrl[10] = positions[7]

            # Lower legs
            data.ctrl[2] = positions[8]
            data.ctrl[5] = positions[9]
            data.ctrl[8] = positions[10]
            data.ctrl[11] = positions[11]

            last_control_time = data.time

        #
        # Step simulation and sleep remaining time to pace
        #
        mujoco.mj_step(model, data)

        viewer.sync()