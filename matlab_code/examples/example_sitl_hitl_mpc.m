%% Call the initialization file for closed loop swarming mpc

cl_init


%% Initialize ROS

ros_init


%% Call the run file to simulate swarming
% Reset is_simulation variable in this file if you need switch from sitl
% to hitl or viceversa.

ros_cl_run


%% Call the analysis file to evaluate the swarm performance

cl_analyze
