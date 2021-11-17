%% Arguments

import casadi.*

% Import swarming param, in S structure
run('/home/esoria/Developer/swarmlab/parameters/param_swarm/param_swarm.m');

% Overwrite and add some param
p_swarm.nb_agents = 5;
p_swarm.max_neig = 2;
p_swarm.max_a = 2;
p_swarm.d = 5;

% Rename param
N = p_swarm.nb_agents; % nb of agents
max_neig = p_swarm.max_neig; % number of neighbours
v_ref = p_swarm.v_ref;
u_ref = p_swarm.u_ref;
d_ref = p_swarm.d_ref;
max_a = p_swarm.max_a;
r_coll = p_swarm.r_coll;
r_comm = p_swarm.r;

rng(4)
position = rand(3*N,1);
pos = SX(position);

pos_matrix = reshape(position, 3, []);
dist = pdist(pos_matrix');
dist_matrix = squareform(dist);

disp(dist_matrix)

%% Copy-paste separation cost without neig restriction

% For every agent define the nonlinear_ls terms
sym_sep = SX.zeros(N*(N-1),1);
for agent = 1:N
    
    % Get the index triplet related to the current agent
    agent_idx = [1,2,3]' + 3*(agent-1)*ones(3,1);
    
    % For every neighbor, compute the distance to the current agent
    for j = 1:(N-1)
        if j < agent
            neig = j;
        else
            neig = j+1;
        end
        neig_idx = [1,2,3]' + 3*(neig-1)*ones(3,1);
        % Separation term
        pos_rel = pos(neig_idx)-pos(agent_idx);
        sym_sep((agent-1)*(N-1)+j) = 1/(N-1)*(pos_rel'*pos_rel - d_ref^2);
    end
end

disp('---------- Output for model WITHOUT neig restriction ----------')
disp(sym_sep)


%% Copy-paste separation cost with neig restriction

[~, sorted_neig] = compute_closest_neighbors(pos, r_comm, max_neig);

% For every agent define the nonlinear_ls terms
for agent = 1:N
    
    % Get the index triplet related to the current agent
    agent_idx = [1,2,3]' + 3*(agent-1)*ones(3,1);
    neigs = sorted_neig(:,agent);
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
        pos_rel(1) = conditional(neig_idx_x, pos_rel_cell, pos_rel_default(1), false);
        pos_rel(2) = conditional(neig_idx_y, pos_rel_cell, pos_rel_default(2), false);
        pos_rel(3) = conditional(neig_idx_z, pos_rel_cell, pos_rel_default(3), false);
        sym_sep((agent-1)*max_neig+j) = 1/max_neig*(pos_rel'*pos_rel - d_ref^2);
        % TODO: Replace max_neig with the current number of neighbours
    end
end

disp('---------- Output for model WITH neig restriction ----------')
disp(sorted_neig)
disp(sym_sep)
