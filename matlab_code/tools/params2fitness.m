function perf = params2fitness( params, d_ref, v_ref, map_size, ...
    nb_blocks, building_width, W_perf, results_path)

%     p_rep       = params(1);
%     r0_fric     = params(2);
%     C_fric      = params(3);
%     v_fric      = params(4);
%     p_fric      = params(5);
%     a_fric      = params(6);
%     r0_shill    = params(7);
%     v_shill     = params(8);
%     p_shill     = params(9);
%     a_shill     = params(10);

%% Generate parameter structures

%%%%%% p_sim structure
addpath(genpath('/home/esoria/Developer/swarmlab'));
p_sim.dt = 0.01;
p_sim.dt_plot = 0.1;
p_sim.start_time = 0;

DRONE_TYPE = "point_mass";
run('param_battery');
run('param_physics');
run('param_drone');

%%%%%% map structure
map = [];
map = generate_map(map, map_size, nb_blocks, building_width);

%%%%%% p_swarm structure
seed_x = floor(rand * 10000);
P0 = [0,0,-1.75]';
P_cube = 1;
p_swarm = generate_vasarhelyi_swarm(seed_x, P0, P_cube, d_ref, v_ref, params, map);

% Run vasarhelyi algorithm
[order, safety, safety_obs] = run_vasarhelyi_swarming( DRONE_TYPE, ...
    p_sim, B, p_swarm, map, results_path );

% Compute simulation performance functions
tot_order       = sum(order,'omitnan')/length(order);
tot_safety      = sum(safety,'omitnan')/length(safety);
tot_safety_obs  = sum(safety_obs,'omitnan')/length(safety_obs);

% Combine into fitness: sum contributions and invert min/max
% The fitness is minimised: the smaller the better.
perf = -(W_perf(1)*tot_order + W_perf(2)*tot_safety + ...
    W_perf(3)*tot_safety_obs);


end