%% Initialize ROS if not done yet

% ros_init


%% Create the Crazyswarm

nb_agents = 1;
is_simulation = true;
crazyswarm = Crazyswarm(is_simulation);
crazyswarm.add_n(nb_agents,swarm_node);
pause(2);


%% Takeoff

height = 1; % meters
duration = 3; % seconds
crazyswarm.takeoff(height, duration, rate);


%% Send velocity commands

iter = 30;
for i = 1:iter
    vx = sin(2*pi/30*i);
    crazyswarm.send_vel_commands(repmat([vx 0 0]',1,nb_agents), rate);
end

% for i = 1:iter
%     crazyswarm.send_vel_commands(repmat([1.5 0 0]',1,nb_agents), rate);
% end

%% Land

duration = 3; % seconds
crazyswarm.land(duration, rate);


%% Plot

figure; plot(crazyswarm.agents(1).vel_cmd_history(:,1));
for i = 1:nb_agents
    hold on; plot(crazyswarm.agents(i).vel_neu_history(:,1));
end
legend('vx cmd','vx');