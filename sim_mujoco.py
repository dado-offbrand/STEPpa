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

for i in range(len(data.ctrl)):
    print(i)
    print(data.ctrl[i])
    if (i % 3 == 0):
        data.ctrl[i] = np.deg2rad(0)
    else:
        data.ctrl[i] = np.deg2rad(90)

    # TODO:
    # Init pose from quadruped's array, not this manual stuff.
    # Continue to adjust kp/kv. Limbs are too slow to ease into final position
    # Why does it seem like joint positions affect other joint positions when moving?
    # Test cycloidal trajectory on one leg ig

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation running. Close the viewer window to exit.")
    
    # 3. Simulation loop
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data)
        viewer.sync()

        # code area

        #quad.gait_planner.cycle()

        # end code area

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
