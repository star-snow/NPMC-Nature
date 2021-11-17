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
% Speed parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Migration direction
p_swarm.u_ref = [1 0 0]';

% Migration speed
p_swarm.v_ref = 0.5;

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

p_swarm.P0 	= [0,0,-1.75]'; %                               [m]
p_swarm.P     = 1; %                                        [m]
p_swarm.V0 	= [0,0,0]'; %                                   [m/s]
p_swarm.V 	= 0; %                                          [m/s]

p_swarm.seed = 5;
rng(p_swarm.seed);

p_swarm.Pos0      = p_swarm.P0 + p_swarm.P * rand(3,p_swarm.nb_agents);
p_swarm.Vel0      = p_swarm.V0 + p_swarm.V * rand(3,p_swarm.nb_agents);
