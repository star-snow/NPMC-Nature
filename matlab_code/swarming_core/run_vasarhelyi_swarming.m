 function [time_history, pos_history, vel_history, ...
    accel_history, dirname] = run_vasarhelyi_swarming( ...
    DRONE_TYPE, p_drone, p_battery, p_swarm, p_sim, p_physics, map, results_path, end_line)

%% Set options

SWARM_ALGORITHM = "vasarhelyi"; % either vasarhelyi or olfati_saber

date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
dirname = strcat(results_path,date_string);
if ~exist(dirname, 'dir')
    mkdir(dirname)
end

fontsize = 12;

%% Set parameters

p_swarm.d_ref = p_swarm.r0_rep;

%% Init Swarm object, Wind, Viewer and other variables

% Init swarm and set positions
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;
for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics, map);
end
swarm.set_pos(p_swarm.Pos0);

% Init wind
wind = zeros(6,1);

% Init variables for history
x0 = [p_swarm.Pos0; zeros(3,p_swarm.nb_agents)];
x_history(1,:) = x0(:);
time_history(1,1) = p_sim.start_time;

%% Main simulation loop

k = 1;
for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Compute velocity commands from swarming algorithm
    swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);
    
    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

    x_history(k+1,:) = swarm.get_state();
    time_history(k+1,1) = time;
    k = k+1;
    
    % Stop simulation when the swarm reaches the goal position
    pos_swarm = swarm.get_pos_ned();
    if sum(pos_swarm(1,:) > end_line) == swarm.nb_agents
        break;
    end
    
end

%% Analyse swarm state variables

% Get variables and save workspace
pos_history = x_history(:,repmat([true true true false false false],1,p_swarm.nb_agents));
vel_history = x_history(:,repmat([false false false true true true],1,p_swarm.nb_agents));
accel_history = diff(vel_history,1)/p_drone.dt;
          
% Plot state variables
agents_color = swarm.get_colors();
lines_color = [];
plot_state_offline(time_history, pos_history, vel_history, ...
    accel_history, agents_color, p_swarm, map, fontsize, lines_color, dirname);

if ~isempty(dirname)
    wokspace_path = strcat(dirname,'/workspace');
    save(wokspace_path, 'time_history', 'pos_history', 'vel_history', ...
    'accel_history', 'p_drone', 'p_sim', 'p_swarm', 'map');
end


end