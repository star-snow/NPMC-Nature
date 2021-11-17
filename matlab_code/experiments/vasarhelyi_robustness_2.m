% Runs the adaptability test of the pf swarm moodel for varying inter-agent
% distances and speeds in the large-scale environment.


%% Parameters

nb_blocks = [3]'; % number of obstacle blocks on the map
d_ref = [0.8]';
v_ref = [0.5 1 1.5]';
nb_iter = 10;

map_size = 8;
end_line = 10;
bulding_width = 0.7;

P0 = [0 0 -1.75]';
P_cube = 1;

% Parameter vector lengths
blocks_len = length(nb_blocks);
d_len = length(d_ref);
v_len = length(v_ref);

% Initialize performance
tot_order = zeros(blocks_len,d_len,v_len);
tot_safety = zeros(blocks_len,d_len,v_len);
tot_safety_obs = zeros(blocks_len,d_len,v_len);

opt_params = [0.2900, 6.9800, 0.0600, 0.6300, 3.340, 0.0500, ... 
    0.1000, 0.8100, 2.9900, 1.1700];

%% Add swarm project for plotting functions

addpath(genpath('/home/esoria/Developer/swarmlab'));


%% Run simulations 

for iter = 1:nb_iter
    for i = 1 : blocks_len
        for j = 1 : d_len
            for k = 1 : v_len

                close all;
                
                %%%%%%%% p_sim structure
                p_sim.dt = 0.01;
                p_sim.dt_plot = 0.1;
                p_sim.start_time = 0;
                p_sim.end_time = 120;

                DRONE_TYPE = "point_mass";
                run('param_battery');
                run('param_physics');
                run('param_drone');
                
                %%%%%%%% Import map param, in map structure
                map = [];
                map = generate_map(map, map_size, nb_blocks(i), bulding_width);
                % run('./params/param_map.m');
                
                %%%%%%%% Import swarm param, in p_swarm structure
                % seed_pos = 5;
                seed_pos = floor(rand * 10000);
                p_swarm = generate_vasarhelyi_swarm(seed_pos, P0, P_cube, d_ref(j), v_ref(k), opt_params, map);
                
                
                %% Run vasarhelyi algorithm
                
                resutls_path = strcat('results/vasarhelyi/robustness_2/', ...
                                num2str(k),'_v_ref/');
                [time_history, pos_history, vel_history, accel_history, dirname] = ...
                    run_vasarhelyi_swarming(DRONE_TYPE, p_sim, p_battery, p_swarm, p_sim, p_physics, map, resutls_path, end_line);
                
                
                %% Compute and plot swarm fitness
                
                [order, safety, safety_obs, min_d_obs, dist_error, speed_error, dir_error, ...
                    speed_delta ] = compute_swarm_fitness(pos_history, vel_history, ...
                    accel_history, p_swarm, dirname);
                
                plot_swarm_fitness(time_history, order, safety, safety_obs, ...
                    min_d_obs, dist_error, speed_error, dir_error, speed_delta, ...
                    p_swarm, dirname);
                
                
                %% Compute simulation performance
                
                tot_order(i,j,k)       = sum(order,'omitnan')/length(order);
                tot_safety(i,j,k)      = sum(safety,'omitnan')/length(safety);
                tot_safety_obs(i,j,k)  = sum(safety_obs,'omitnan')/length(safety_obs);

                
            end
        end
    end
end


