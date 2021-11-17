% Runs the adaptability test of the pf swarm moodel for varying obstacle
% densities in the large-scale scenario.

%% Parameters

nb_blocks = [3 4 5]'; % number of obstacle blocks on the map
d_ref = [7]';
v_ref = [3]';
nb_iter = 1;

map_size = 100;
end_line = 130;
bulding_width = 12;

P0 = [0, 0, -40]';
P_cube = 8;

% Parameter vector lengths
blocks_len = length(nb_blocks);
d_len = length(d_ref);
v_len = length(v_ref);

% Initialize performance
tot_order = zeros(blocks_len,d_len,v_len);
tot_safety = zeros(blocks_len,d_len,v_len);
tot_safety_obs = zeros(blocks_len,d_len,v_len);

opt_params = [0.1133,  80.2405,   0.0403,   0.5866,  2.8178,   2.5812, ...
    0.2508,   7.5653,   3.1088,   2.1146];


%% Add swarm project for plotting functions

addpath(genpath('/home/esoria/Developer/swarmlab'));


%% Run simulations 

for iter = 1:nb_iter
    for i = 1 : blocks_len
        for j = 1 : d_len
            for k = 1 : v_len
                
                close all;
                
                %%%%%%%% P structure
                p_sim.dt = 0.01;
                p_sim.dt_plot = 0.1;
                p_sim.start_time = 0;
                p_sim.end_time = 220;

                DRONE_TYPE = "point_mass";
                run('param_battery');
                run('param_physics');
                run('param_drone');
                
                %%%%%%%% Import map param, in map structure
                map = [];
                map = generate_map(map, map_size, nb_blocks(i), bulding_width);
                
                %%%%%%%% Import swarm param, in p_swarm structure
                seed_pos = floor(rand * 10000);
                p_swarm = generate_vasarhelyi_swarm(seed_pos, P0, P_cube, d_ref(j), ...
                    v_ref(k), opt_params, map);
                
                
                %% Run vasarhelyi algorithm
                
                resutls_path = strcat('results/vasarhelyi/adaptability_2/', ...
                                num2str(nb_blocks(i)),'_blocks/');
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