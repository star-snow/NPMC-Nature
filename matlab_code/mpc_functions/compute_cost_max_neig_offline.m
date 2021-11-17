function cost = compute_cost_max_neig_offline(p_swarm, model, pos_history, vel_history, u_history)
% compute_cost_offline - gives in output a vector with the values
% of the MPC cost function computed at every time step.

%% Rename parameters

% Swarming
N = p_swarm.nb_agents;          % number of agents
max_neig = p_swarm.max_neig;    % number of neighbours
v_ref = p_swarm.v_ref;          % reference speed for all agents
u_ref = p_swarm.u_ref;          % reference direction of velocity for all agents
d_ref = p_swarm.d_ref;          % reference distance among every couple of neighboring agents
max_a = p_swarm.max_a;          % maximum allowed acceleration
r_comm = p_swarm.r;             % radius of communication
r_coll = p_swarm.r_coll;        % collision radius of an agent

% Weights
W_sep = model.W_sep;
W_dir = model.W_dir;
W_nav = model.W_nav;
W_u = model.W_u; % Penalization of high values of the control input variables
W = model.W;


%% Main loop to compute the MPC cost

[nb_steps,~] = size(pos_history(1:(end-1),:));
cost = zeros(nb_steps,1);



for step = 1:nb_steps
    
    sep_cost = 0;
    dir_cost = 0;
    nav_cost = 0;
    u_cost = 0;
    
    pos = (pos_history(step,:))';
    Pos_k = reshape(pos,3,[]);
    Dist_k = pdist(Pos_k');
    Dist_k_matrix = squareform(Dist_k);
    vel = (vel_history(step,:))';
    u = (u_history(step,:))';
    
    % Neighborhood matrix
    % M = ones(N,N) - eye(N,N);
    M = compute_neighborhood(Dist_k_matrix, r_comm, max_neig);
    
    % For every agent define the nonlinear_ls terms
    for agent = 1:N
        
        % Get the index triplet related to the current agent
        agent_idx = [1,2,3]' + 3*(agent-1)*ones(3,1);
        neigs = find(M(:,agent));
        
        % For every neighbor, compute the distance to the current agent
        for j = 1 : max_neig
            neig = neigs(j);
            neig_idx = [1,2,3]' + 3*(neig-1)*ones(3,1);
            
            % Separation term
            pos_rel = pos(neig_idx)-pos(agent_idx);
            sep_cost = sep_cost + (pos_rel'*pos_rel - d_ref^2)/max_neig;
        end
        
        vel_agent = vel(agent_idx);
        u_agent = u(agent_idx);
        
        % Direction term
        dir_cost = dir_cost + (1 - (vel_agent'*u_ref)^2/(vel_agent'*vel_agent));
        
        % Navigation term
        nav_cost = nav_cost + (vel_agent'*vel_agent - v_ref^2);
        u_cost = u_cost + u_agent'*u_agent;
    end
    
    % Assemble expr_y
    cost(step) = sum([W_sep*sep_cost^2, W_dir*dir_cost^2, W_nav*nav_cost^2, W_u*u_cost],'omitnan');
    
end

end