# human-robot-collider
This software package simulates and analyses collisions between a walking pedestrian model and a mobile robot to investigate the risk associated to such human-robot collisions and to support safety-guided robot design. The software lets both agents approach each other and collide with different angles and speeds, using the simulation engine Bullet (via pybullet) to detect the potential contact locations on both agents' bodies. Phase 1 denotes this step. Next, it allows to further analyse the detected contacts by simulating the dynamical interaction forces using a planar model with a linear spring connecting the colliding human limb (approximated as a point mass) and the robot (modeled as a rigid body with a differential drive constraint). We denote this step as phase 2. Finally, it serves to investigate how different controller parameters, namely the robot's braking force and braking delay, affect the peak force statistics over all the collision samples. We term the latter analysis and evaluation as phase 3.

## Prerequisites
Phase 1 requires to install the simulation engine pybullet. On Linux, one can do so by simply executing in a terminal the following command.
```
pip3 install -U pybullet
```
Phase 2 and 3 require MATLAB.

## Executing Phase 1
To run and visualize phase 1 for example for an adult pedestrian, one can execute the following commands.
```
cd path/to/human-robot-collider/script
python3 hrc.py adult gui 
```
Replacing adult by child in the above command will run the same for a child pedestrian. Replacing gui with no_gui will run everything as fast as possible without any visualization. In any case the result is stored in the files result_phase_1.mat and result_phase_1.npy. 

![Alt text](/docs/snapshots.png?raw=true "Phase 1 snapshots from the visualization.")


## Executing Phase 2

After executing Phase 1, one can execute phase 2 by opening MATLAB and running the script control_simulation.m, which loads the output of phase 1 from the file result_phase_1.mat and initializes and carries out a separate contact simulation for each contact reported by phase 1.

## Executing Phase 3

The MATLAB script Delays_simulation.m compares the effect of delays and risetime in the braking force.