function model = swarming_model_max_neig(p_swarm)

% swarming_model_max_neig - Function that describes the dynamics of the 
% swarm and the cost function for controlling it. This model considers only
% closest neighbors (up to a maximum number).
%
% Swarming  is the behavior of collective and ordered motion. It can be 
% obtained through the combination of the following rules:
%
% Separation rule: drives the agents to a reference inter-agent ...
%       distance (d_ref)
% Direction rule: make the agents' velocity converge to a ...
%       reference direction (u_ref)
% Navigation (or propulsion) rule: make the agents' speed converge to a ...
%       reference value (v_ref)
%


import casadi.*


%% Swarm and obstacles parameters

% Swarm params
N = p_swarm.nb_agents;          % number of agents
max_neig = p_swarm.max_neig;    % number of neighbours
v_ref = p_swarm.v_ref;          % reference speed for all agents
u_ref = p_swarm.u_ref;          % reference velocity direction for all agents (unit vector)
d_ref = p_swarm.d_ref;          % reference distance among every couple of neighboring agents
max_a = sqrt(p_swarm.max_a^2/3);% maximum allowed acceleration
r_comm = p_swarm.r;             % radius of communication
r_coll = p_swarm.r_coll;        % collision radius of an agent

% Obstacles params
nb_cylinders = p_swarm.n_cyl;
if nb_cylinders > 0
    c_obstacles = p_swarm.cylinders(1:2,:);
    r_obstacles = p_swarm.cylinders(3,:);
    M_obs = ones(nb_cylinders, N);
end

% Safety margin for agent-obstacle collisions
safety_margin = 0.2;


%% System dimensions

nx = 6 * N; % nb of state variables
nu = 3 * N; % nb of control inputs


%% Named symbolic variables

pos = SX.sym('p', 3*N); % 3D positions of the agents [m]
vel = SX.sym('v', 3*N); % 3D velocities of the agents [m/s]
u = SX.sym('a', 3*N);   % 3D acceleration to apply to agents [m/s^2]


%% Unnamed symbolic variables

sym_x = vertcat(pos, vel);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = u;


%% Dynamics

expr_f_expl = vertcat(vel, ...
                      u);
expr_f_impl = expr_f_expl - sym_xdot;


%% Nonlinear least squares

% Weights
W_sep = 1; 
W_dir = 5;
W_nav = 5;
W_u = 4e-1; % Penalization of high values of the control input variables

sym_sep = SX.zeros(N*max_neig,1);
sym_dir = SX.zeros(N,1);
sym_nav = SX.zeros(N,1);


%% Compute neighbors

% Uncomment in case no restriction on neighbors apply
% aux_matrix = repmat((1:N)',1,N)
% upper = triu(aux_matrix,1);
% lower = tril(aux_matrix,-1);
% neig_matrix = upper(1:(N-1),:) + lower(2:N,:);

[~, neig_matrix] = compute_closest_neighbors(pos, r_comm, max_neig);


%% For every agent define the cost function terms

for agent = 1:N
    
    % Get the index triplet related to the current agent
    agent_idx = [1,2,3]' + 3*(agent-1)*ones(3,1);
    neigs = neig_matrix(:,agent);
    
    % For every neighbor, compute the distance to the current agent
    for j = 1:max_neig
        neig = neigs(j);
        neig_idx = [1,2,3]' + 3*(neig-1)*ones(3,1);
        
        % Separation term
        pos_rel_cell = vertsplit(pos-repmat(pos(agent_idx),N,1));
        pos_rel_default = [d_ref;0;0];
        pos_rel = SX.zeros(3,1);
        neig_idx_x = neig_idx(1)-1;
        neig_idx_y = neig_idx(2)-1;
        neig_idx_z = neig_idx(3)-1;
        
        % Conditional method only accept scalar index
        pos_rel(1) = conditional(neig_idx_x, pos_rel_cell, pos_rel_default(1), false);
        pos_rel(2) = conditional(neig_idx_y, pos_rel_cell, pos_rel_default(2), false);
        pos_rel(3) = conditional(neig_idx_z, pos_rel_cell, pos_rel_default(3), false);
        sym_sep((agent-1)*max_neig+j) = 1/max_neig*(pos_rel'*pos_rel - d_ref^2);
    end
    
    vel_agent = vel(agent_idx);
    
    % Direction term
    sym_dir(agent) = 1 - (vel_agent'*u_ref)^2/(vel_agent'*vel_agent);
    
    % Navigation term
    sym_nav(agent) = vel_agent'*vel_agent - v_ref^2;
    
end

% Weights
W = [W_sep/(max_neig)*ones(N*(max_neig),1); W_dir*ones(N,1); ...
    W_nav*ones(N,1); W_u*W_sep*ones(3*N,1)];
W_e = [W_sep*ones(N*(max_neig),1); W_dir*ones(N,1); ...
    W_nav*ones(N,1)];

% Assemble expr_y
expr_y = vertcat(sym_sep, sym_dir, sym_nav, W_u*sym_u);
expr_y_e = vertcat(sym_sep, sym_dir, sym_nav);

ny = length(expr_y);
ny_e = length(expr_y_e);


%% Constraints

%%%%%%%%% Constraints to avoid agent-agent collisions 
sym_dist = SX.zeros(N*(N-1)/2,1);
nh_agents = 0;

for agent = 1:(N-1)
    agent_idx = [1,2,3]' + 3*(agent-1)*ones(3,1);
    for neig = (agent+1):N
        nh_agents = nh_agents +1;
        neig_idx = [1,2,3]' + 3*(neig-1)*ones(3,1);
        pos_rel = pos(neig_idx)-pos(agent_idx);
        sym_dist(nh_agents) = pos_rel(1)^2+pos_rel(2)^2+pos_rel(3)^2;
    end
end

%%%%%%%%% Constraints to avoid agent-obstacle collisions
sym_dist_obs = SX.zeros(N*nb_cylinders,1);
nh_obs = 0;
lh_obs_coll = zeros(N*nb_cylinders,1);
uh_obs_coll = zeros(N*nb_cylinders,1);

if nb_cylinders > 0
    for agent = 1:N
        agent_idx = [1;2] + 3*(agent-1)*ones(2,1);
        neig_obs = M_obs(:,agent);
        nb_neig_obs = sum(neig_obs);
        if sum(neig_obs) > 0
            for j = 1:nb_neig_obs
                nh_obs = nh_obs+1;
                pos_rel_obs = pos(agent_idx) - c_obstacles(:,j);
                sym_dist_obs(nh_obs) = pos_rel_obs(1)^2 + ...
                    pos_rel_obs(2)^2 - r_obstacles(j)^2;
                lh_obs_coll(nh_obs) = safety_margin^2;
                uh_obs_coll(nh_obs) = 1e16;
            end
        end
    end
end

% Gather contraints expressions in a single vector
expr_h = vertcat(sym_u, ...
                 sym_dist, ...
                 sym_dist_obs);
nh = nu + nh_agents + nh_obs;
% expr_h_e = sym_x;

% Constraint bounds
lh_u = - max_a * ones(nu, 1);
uh_u = max_a * ones(nu, 1);
lh_agent_coll = 4 * r_coll^2 * ones(nh_agents,1);
uh_agent_coll = 300^2 * ones(nh_agents,1);

if nb_cylinders > 0
    lh = [lh_u; lh_agent_coll; lh_obs_coll];
    uh = [uh_u; uh_agent_coll; uh_obs_coll];
else
    lh = [lh_u; lh_agent_coll];
    uh = [uh_u; uh_agent_coll];
end


%% Populate structure

model.nx = nx;
model.nu = nu;
model.ny = ny;
model.ny_e = ny_e;
model.nh = nh;
model.nh_e = 0;
model.nh_e = 0;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
% model.expr_h_e = expr_h_e;
model.expr_y = expr_y;
model.expr_y_e = expr_y_e;
model.lh = lh;
model.uh = uh;
% model.lh_e = zeros(nh_e, 1);
% model.uh_e = zeros(nh_e, 1);

model.W_sep = W_sep; 
model.W_dir = W_dir;
model.W_nav = W_nav;
model.W_u = W_u;
model.W = W;
model.W_e = W_e;
