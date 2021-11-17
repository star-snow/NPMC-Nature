%% Set options

close all;
clearvars -except app;

% Add swarmlab project to path
addpath(genpath('/home/esoria/Developer/swarmlab'));

DRONE_TYPE = "point_mass"; % swarming mode supports only quadcopter and point_mass
SWARM_ALGORITHM = "vasarhelyi"; % either vasarhelyi or olfati_saber

date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
dirname = strcat('results/vasarhelyi/',date_string);
if ~exist(dirname, 'dir')
    mkdir(dirname)
end

fontsize = 10;


%% Set parameters

map = [];

run('param_sim');
p_sim.end_time = 120;
p_sim.dt_video = 0.1;
p_sim.dt_plot = 0.1;

run('param_battery');
run('param_physics');
run('param_drone');

run('./params/param_map'); % creates map: struct for the map parameters
run('./params/param_swarm');
run('./params/param_vasarhelyi');

p_swarm.d_ref = p_swarm.r0_rep;


%% Init Swarm object and other variables

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

goal_reached = false;
k = 1;

for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Compute velocity commands from swarming algorithm
    [vel_c,~] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);
    
    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

    x_history(k+1,:) = swarm.get_state();
    time_history(k+1,1) = time;
    k = k+1;
    
    % Stop simulation when the swarm reaches the goal position
    pos_swarm = swarm.get_pos_ned();
    if sum(pos_swarm(1,:) > map.width) == swarm.nb_agents
        break;
    end
    
end


%% Analyse swarm state variables

% Get variables and save workspace
pos_history = x_history(:,repmat([true true true false false false],1,p_swarm.nb_agents));
vel_history = x_history(:,repmat([false false false true true true],1,p_swarm.nb_agents));
accel_history = diff(vel_history,1)/p_sim.dt;
          
% Plot state variables
agents_color = swarm.get_colors();
lines_color = [];
plot_state_offline(time_history, pos_history, vel_history, ...
    accel_history, agents_color, p_swarm, map, fontsize, lines_color, dirname);


%% Analyse performance

% Compute swarm performance
[safety, order, union, alg_conn, safety_obs, min_d_obs ] = ...
    compute_swarm_performance(pos_history, vel_history, ...
    p_swarm, dirname);

% Plot performance
[perf_handle] = plot_swarm_performance(time_history, safety, order, ...
    union, alg_conn, safety_obs, min_d_obs, p_swarm, fontsize, dirname);

if ~isempty(dirname)
    wokspace_path = strcat(dirname,'/workspace');
    save(wokspace_path);
end
