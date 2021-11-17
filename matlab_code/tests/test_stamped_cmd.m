%% Initialize ROS if not done yet

% ros_init


%% Create the Crazyswarm

is_simulation = true;
crazyswarm = Crazyswarm(is_simulation);
crazyswarm.add_n(5,swarm_node);
pause(2);

rostime('now','system')
headers = crazyswarm.continuous_vel_commands([-0.5 0 0]', 0.1, rate);
rostime('now','system')