%% Initialize ROS if not done yet

% ros_init
nb_agents = 1;

%% Create the Crazyswarm

is_simulation = true;
crazyswarm = Crazyswarm(is_simulation);
crazyswarm.add_n(nb_agents,swarm_node);
pause(2);


%% Takeoff

height = 1; % meters
duration = 3; % seconds
crazyswarm.takeoff(height, duration, rate);

%% Go to initial position

duration = 2; % seconds
% NED to NEU
current_positions = crazyswarm.get_positions();
goal_positions = [current_positions(1,:); current_positions(2,:); height*ones(1,nb_agents)];
% goal_positions = [0 1; 4 4; 1 1];
crazyswarm.go_to(goal_positions, duration, rate);


%% Send velocity commands

iter = 120;
tic
current_positions = crazyswarm.get_positions();
for i = 1:iter
    dpx = sin(2*pi/60*i);
    dp = repmat([dpx 0 0]',1,nb_agents);
    pos_commands = current_positions+dp;
    crazyswarm.send_pos_commands(pos_commands, rate);
end
toc

% tic;
% current_positions = crazyswarm.get_positions();
% for i = 1:iter
%     dpx = 0.1*i;
%     dp = repmat([dpx 0 height]',1,nb_agents);
%     pos_commands = current_positions+dp;
%     crazyswarm.send_pos_commands(pos_commands, rate);
% end
% toc;

%% Land

duration = 3; % seconds
crazyswarm.land(duration, rate);


%% Plot

time_end = length(crazyswarm.agents(1).pos_neu_history(:,1));
time_history = (1:time_end)'*dt;
figure; plot(time_history, crazyswarm.agents(1).pos_cmd_history(:,1)-current_positions(1,1));
for i = 1:nb_agents
    hold on; plot(time_history, ...
        crazyswarm.agents(i).pos_neu_history(:,1)-current_positions(1,i));
end
legend('px cmd','px');
xlabel('Time [s]');
ylabel('X position [m]')