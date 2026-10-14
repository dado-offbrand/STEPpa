**STEPpa, a robotic quadruped**<br>

This is STEPpa, a 12DOF generic robotic quadruped. I started this project with a fascination in quadrupeds, and I have no interest in
making this an open-source maintained project. My files are publicly available here though, so feel free to wonder around! See the notes below, as it
is pretty much the only "documentation" that exists in this repository for information about my quadruped.<br>

**Quadruped specs:**<br>
Motors: Steadywin GIM6010-8, 5Nm continuous, 11Nm peak, 24v nominal<br>
Motor Controllers: Moteus r4.11, awaiting release of r5 to convert my quadruped to these instead.<br>
Battery: Milwaukee M18 drill battery (a high output/capacity variant)<br>
Compute: RPI4 with pi3hat from MJbots<br>
Power distribution: power_dist from MJbots<br>
Weight: TBD, still adding components.<br>
Lower leg actuation method: Tensioned GT3 belt<br>

**Quadruped programming:**<br>
The main quadruped class has four legs defined, which all have trajectory planners defined. In order to fully cycle the movement and get the angles
for each motor, I update the gait which updates each trajectory planner based on the current gait, then I can get an array with all of the angles
in a specified order. This was made to be very modular and separated whichis why there are so many components, but through development, many old 
functions have been left and the code is less efficient than it could be, especially with it being in python. In the future I'll likely convert 
to C++ for the IK functions, and maybe try rust just for giggles.<br>

**Simulation Environment:**<br>
I used Nvidia's Isaac Sim before switching to MuJoCo. I left the Isaac Sim assets in case I need them in the future, although, I do not think I will be doing
reinforcement learning anytime soon.<br>

**CAD notes:**<br>
A lot of people have asked for the CAD files, and I myself have done that to other people posting their quadrupeds. From my own experience, having a CAD file
to reference is nice, especially when recreating real-world hardware you don't have the STEP files for. I don't mind if people inspire their designs off of
my mechanisms, but I want my quadruped to be unique and designed by me, so I don't want to release the full STEP files or CAD assembly of my quadruped for people to just create a 1:1 clone of. It's weird to gatekeep it, but it has been a lot of hard work. If
anybody is somehow reading this and cannot understand how to recreate a certain mechanism, feel free to reach out to me at my instagram @ben_.ward<br>
