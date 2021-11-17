%% Setup ROS

addpath(genpath('/home/esoria/Developer/mat-ros'));

% rosinit('http://lisB086184:11311/')
rosinit('http://localhost:11311')

% Create node
swarm_node = ros.Node('/mpc_swarm');

freq = 1/dt;
rate = rosrate(freq);
reset(rate)


%% Set simulation argument (true/false)
% true: to run sitl simulations in gazebo
% false: to run it hitl experiments with crazyflies

is_simulation = false;
