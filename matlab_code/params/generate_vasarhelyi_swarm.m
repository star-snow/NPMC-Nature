function p_swarm = generate_vicsek_swarm(seed_pos, P0, P_cube, d_ref, v_ref, params, map)

% Variables to be set
p_swarm.is_active_migration = true;
p_swarm.is_active_goal = false;
p_swarm.is_active_arena = false;
p_swarm.is_active_spheres = false;
p_swarm.is_active_cyl = true;
p_swarm.draw_plot = false;
p_swarm.draw_state_var = false;
p_swarm.draw_neig_links = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.nb_agents = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max radius of influence - Metric distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r = 150;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max number of neighbors - Topological distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.max_neig = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max field of view
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% P.aov = 120;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radius of collision
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r_coll = 0.1;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylindric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') & ~isempty(map))

    nb_obstacles = length(map.buildings_east);
    cylinder_radius = map.building_width / 2;

    p_swarm.cylinders = [
        map.buildings_north'; % x_obstacle
        map.buildings_east'; % y_obstacle
        repmat(cylinder_radius, 1, nb_obstacles)]; % r_obstacle
    p_swarm.n_cyl = length(p_swarm.cylinders(1, :));
else
    p_swarm.cylinders = 0;
    p_swarm.n_cyl = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Speed and distance parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Migration direction
p_swarm.u_ref = [1 0 0]';

% Migration speed
p_swarm.v_ref = v_ref;

% Rerence inter-agent distance
p_swarm.d_ref = d_ref;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity and acceleration bounds for the agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.max_a = 2;
p_swarm.max_v = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position and velocity for the swarm
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.P0 	= P0; %                                         [m]
p_swarm.P     = P_cube; %                                     [m]
p_swarm.V0 	= [0,0,0]'; %                                   [m/s]
p_swarm.V 	= 0; %                                          [m/s]
p_swarm.seed = seed_pos;

rng(p_swarm.seed);
p_swarm.Pos0      = p_swarm.P0 + p_swarm.P * rand(3,p_swarm.nb_agents) - [0,P_cube/2,0]';

% Change initial position if any collision happen
dist_m      = dist(p_swarm.Pos0);
nb_coll      = sum(sum(((dist_m < 2*p_swarm.r_coll) & (dist_m ~= 0))));
while nb_coll ~= 0
    p_swarm.seed = p_swarm.seed + 1;
    p_swarm.Pos0      = p_swarm.P0 + p_swarm.P * rand(3,p_swarm.nb_agents) - [0,P_cube/2,0]';
    dist_m      = dist(p_swarm.Pos0);
    nb_coll     = sum(sum(((dist_m < 2*p_swarm.r_coll) & (dist_m ~= 0))));
end

p_swarm.Vel0      = p_swarm.V0 + p_swarm.V * rand(3,p_swarm.nb_agents);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vicsek Paramteres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% Repulsion %%%%%%%%%%%%%%%%%%%%%%
%
% Repulsion range
p_swarm.r0_rep = d_ref; % 25.6;
% Repulsion gain
p_swarm.p_rep = params(1); % 0.13;

%%%%%%%%%%%%%%%%%% Friction %%%%%%%%%%%%%%%%%%%%%%
%
% Stopping point offset of alignment
p_swarm.r0_fric = params(2); %85.3
% Coefficient of velocity alignment
p_swarm.C_fric = params(3);
% Velocity slack of alignement
p_swarm.v_fric = params(4);
% Gain of braking curve
p_swarm.p_fric = params(5);
% Acceleration of braking curve
p_swarm.a_fric = params(6); %4.16;

%%%%%%%%%%%%%%%%%% Obstacles and wall %%%%%%%%%%%%
%
% Stopping point offset of walls
p_swarm.r0_shill = params(7); % 0.3;
% Velocity of virtual shill agents
p_swarm.v_shill = params(8); % 13.6;
% Gain of braking curve for walls
p_swarm.p_shill = params(9); % 3.55;
% Acceleration of braking curve for walls
p_swarm.a_shill = params(10); % 3.02


end