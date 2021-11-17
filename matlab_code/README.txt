This folder contains the code that has been used for the submission in Nature Machine Intelligence "Predictive Control of Aerial Swarms in Cluttered Environments".

Folder structure
----------------

The structure of the folder is as follows.

- acados_templates: contains example on how to create and solve an MPC problem with the help of acados (https://docs.acados.org/), a library for fast and embedded solvers for nonlinear optimal control.
 
- examples: contains the scripts that one should call for running the NMPC swarm simulation experiments in MATLAB or experiments with Software In The Loop (SITL) or Hardware In The Loop (HITL). SITL experiments can be performed upon spawning Crazyflie drones in Gazebo, while HITL experiments can be performed using Crazyflies drones (https://store.bitcraze.io/products/crazyflie-2-1) and the open-source project Crazyswarm (https://crazyswarm.readthedocs.io/en/latest/). Other examples files allows to verify the correct takeoff of a fleet of drones in the real world, and contain the simulation of the Vasarhelyi's swarm model [Vasarhelyi et al., Science Robotics, 2018].

- mpc_functions: contains the functions used in the definition of the NMPC swarming problem.

- swarming_core: contains the initialization, run and analysis files that coompose the swarminng experiments. It also contains wrapper classes (Crazyswarm.m, Crazyflie.m) for Matlab-ROS communication with the Crazyflies.

- experiments: contains the scripts for running all experiments.

- params: contains the parameter files for the environment and the swarm.

- tools: contains auxiliary functions for computing statistics, opitmi

- tests: contains test files.


How to run the simulation experiments
-------------------------------------

To run the NMPC simulation experiments you need to:
- clone acados (master branch, commit 91067daebe12c07d76d32a6aed0b8db00b3a54e1)
- clone swarmlab (v1.0, https://github.com/lis-epfl/swarmlab/tree/v1.0)
- add the content of this folder in acados/examples/acados_matlab_octave/swarming
- run the example files 'example_mpc.m'


How to run the hardware experiments
-----------------------------------

To run the NMPC hardware experiments you need to:
- equip your Crazyflies (https://store.bitcraze.io/products/crazyflie-2-1) with markers
- clone the LIS fork of Crazyswarm (https://github.com/lis-epfl/crazyswarm/tree/master-mpc-swarm)
- connect your Motion tracking software
- generate the crazyflie_driver custom messages in Matlab, as in crazyswarm-matlab (https://github.com/lis-epfl/crazyswarm-matlab)
- fly your drones following the example 'example_sitl_hitl_mpc.m'


Please, refer to the requirements.txt file for veryfing the software version used for this project.


