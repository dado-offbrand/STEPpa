import time
import mujoco
import mujoco.viewer
import quadruped
import numpy as np

# URDF loading
model = mujoco.MjModel.from_xml_path('quadruped_mujoco.xml')
data = mujoco.MjData(model)

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
    
    # 3. Simulation loop
    while viewer.is_running():
        mujoco.mj_step(model, data) 

        #
        # Get quadruped position array
        #
        quad.gait_planner.cycle()
        positions = quad.get_pose_array()

        #
        # Set simulation data
        #
        for i in range(len(data.ctrl)):
            data.ctrl[i] = positions[i]
            print(np.rad2deg(positions[i]))
        print("\n\n\n")
        # Shoulders
        # data.ctrl[0] = np.deg2rad(0)
        # data.ctrl[3] = np.deg2rad(0)
        # data.ctrl[6] = np.deg2rad(0)
        # data.ctrl[9] = np.deg2rad(0)

        # # Upper legs
        # data.ctrl[1] = np.deg2rad(45)
        # data.ctrl[4] = np.deg2rad(45)
        # data.ctrl[7] = np.deg2rad(45)
        # data.ctrl[10] = np.deg2rad(45)

        # # Lower legs
        # data.ctrl[2] = np.deg2rad(90)
        # data.ctrl[5] = np.deg2rad(90)
        # data.ctrl[8] = np.deg2rad(90)
        # data.ctrl[11] = np.deg2rad(90)

        # end non-sim code area
        viewer.sync()