**STEPpa, a robotic quadruped**

This is STEPpa, a 12DOF generic robotic quadruped. I started this project with a fascination in quadrupeds, and I have no interest in
making this an open-source maintained project. My files are publicly available here though, so feel free to wonder around! See the notes below, as it
is pretty much the only "documentation" that exists in this repository for information about my quadruped.

**Quadruped specs:**
Motors: Steadywin GIM6010-8, 5Nm continuous, 11Nm peak, 24v nominal
Motor Controllers: Moteus r4.11, awaiting release of r5 to convert my quadruped to these instead.
Battery: Milwaukee M18 drill battery (a high output/capacity variant)
Compute: RPI4 with pi3hat from MJbots
Power distribution: power_dist from MJbots
Weight: TBD, still adding components.
Lower leg actuation method: Tensioned GT3 belt

**Quadruped programming:**
The main quadruped class has four legs defined, which all have trajectory planners defined. In order to fully cycle the movement and get the angles
for each motor, I update the gait which updates each trajectory planner based on the current gait, then I can get an array with all of the angles
in a specified order. This was made to be very modular and separated whichis why there are so many components, but through development, many old 
functions have been left and the code is less efficient than it could be, especially with it being in python. In the future I'll likely convert 
to C++ for the IK functions, and maybe try rust just for giggles.

**Simulation Environment:**
I used Nvidia's Isaac Sim before switching to MuJoCo. I left the Isaac Sim assets in case I need them in the future, although, I do not think I will be doing
reinforcement learning anytime soon.

**CAD notes:**
A lot of people have asked for the CAD files, and I myself have done that to other people posting their quadrupeds. From my own experience, having a CAD file
to reference is nice, especially when recreating real-world hardware you don't have the STEP files for. I don't mind if people inspire their designs off of
my mechanisms, but I want my quadruped to be unique and designed by me, so I don't want to release the full STEP files or CAD assembly of my quadruped. If
anybody is somehow reading this and cannot understand how to create a certain mechanism, feel free to reach out to me at my instagram @ben_.ward
