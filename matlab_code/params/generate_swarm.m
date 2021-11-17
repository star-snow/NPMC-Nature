function S = generate_swarm(seed_pos, P0, P_cube, d_ref, v_ref, map)

% Variables to be set
S.is_active_migration = true;
S.is_active_goal = false;
S.is_active_arena = false;
S.is_active_spheres = false;
S.is_active_cyl = true;
S.draw_plot = false;
S.draw_state_var = false;
S.draw_neig_links = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.nb_agents = 5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.d_ref = d_ref;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max radius of influence - Metric distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.r = 150;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max number of neighbors - Topological distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.max_neig = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max field of view
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% P.aov = 120;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radius of collision
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.r_coll = 0.1;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylindric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') & ~isempty(map))

    nb_obstacles = length(map.buildings_east);
    cylinder_radius = map.building_width / 2;

    S.cylinders = [
        map.buildings_north'; % x_obstacle
        map.buildings_east'; % y_obstacle
        repmat(cylinder_radius, 1, nb_obstacles)]; % r_obstacle
    S.n_cyl = length(S.cylinders(1, :));
else
    S.cylinders = 0;
    S.n_cyl = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Speed parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Migration direction
S.u_ref = [1 0 0]';

% Migration speed
S.v_ref = v_ref;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity and acceleration bounds for the agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.max_a = 2;
S.max_v = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position and velocity for the swarm
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S.P0 	= P0; %                               [m]
S.P     = P_cube; %                                    [m]
S.V0 	= [0,0,0]'; %                                   [m/s]
S.V 	= 0; %                                          [m/s]
S.seed = seed_pos;

% Initial position are generated in a random uniform cube symmetric w.r.t
% the x axis
rng(S.seed);
S.Pos0      = S.P0 + S.P * rand(3,S.nb_agents) - [0,P_cube/2,0]';

% Change initial position if any collision happen
dist_m      = dist(S.Pos0);
nb_coll      = sum(sum(((dist_m < 2*S.r_coll) & (dist_m ~= 0))));
while nb_coll ~= 0
    S.seed = S.seed + 1;
    S.Pos0      = S.P0 + S.P * rand(3,S.nb_agents) - [0,P_cube/2,0]';
    dist_m      = dist(S.Pos0);
    nb_coll     = sum(sum(((dist_m < 2*S.r_coll) & (dist_m ~= 0))));
end

S.Vel0      = S.V0 + S.V * rand(3,S.nb_agents);

end