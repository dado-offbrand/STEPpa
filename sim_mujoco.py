import time
import mujoco
import mujoco.viewer
import quadruped
import numpy as np
import keyboard

# URDF loading
model = mujoco.MjModel.from_xml_path('quadruped_mujoco.xml')
data = mujoco.MjData(model)

# Time guff
CONTROL_INTERVAL_MS = 3.5 # run every x miliseconds
control_dt = CONTROL_INTERVAL_MS / 1000.0  # convert to seconds

# Custom class guff
quad = quadruped.Quadruped()

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Running sim")
    last_control_time = data.time
    last_render_time = time.time()
    quad.gait_planner.forward_vector = 0

    while viewer.is_running():
        step_start = time.time()

        #
        # Set quadruped movement vector
        #
        if (keyboard.is_pressed('up')):
            quad.gait_planner.forward_vector = 1
        elif (keyboard.is_pressed('down')):
            quad.gait_planner.forward_vector = -1
        elif not quad.gait_planner.forward_vector == 0:
            quad.gait_planner.forward_vector = 0

        if (keyboard.is_pressed('left')):
            quad.gait_planner.side_vector = -1
        elif (keyboard.is_pressed('right')):
            quad.gait_planner.side_vector = 1
        elif not quad.gait_planner.side_vector == 0:
            quad.gait_planner.side_vector = 0

        if (keyboard.is_pressed('q')):
            quad.gait_planner.rotational_vector = -1
        elif (keyboard.is_pressed('e')):
            quad.gait_planner.rotational_vector = 1
        elif not quad.gait_planner.rotational_vector == 0:
            quad.gait_planner.rotational_vector = 0

        #
        # Set simulation data every CONTROL_INTERVAL_MS ms
        #
        if (data.time - last_control_time) >= control_dt:
            targets = quad.gait_planner.cycle()
            quad.update_all_poses(targets)
            positions = quad.get_pose_array()

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

        mujoco.mj_step(model, data)

        # So that render isn't called every physics step (60fps)
        current_time = time.time()
        if (current_time - last_render_time) > (1.0 / 60.0):
            viewer.sync()
            last_render_time = current_time

        # So that physics steps aren't a bajillion times a second
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)