%% Initialize ROS if not done yet

% ros_init


%% Create the Crazyswarm

is_simulation = false;
crazyswarm = Crazyswarm(is_simulation);
crazyswarm.add_n(p_swarm.nb_agents, swarm_node);
pause(2);


%% Takeoff

height = 1; % meters
duration = 3; % seconds
crazyswarm.takeoff(height, duration, rate);


%% Land

duration = 3; % seconds
crazyswarm.land(duration, rate);


%% Plot

fig = figure; 
subplot(2,1,1);
plot(crazyswarm.agents(1).pos_cmd_history);
legend('X','Y','Z');

subplot(2,1,2);
plot(crazyswarm.agents(1).pos_neu_history);
legend('X','Y','Z');
