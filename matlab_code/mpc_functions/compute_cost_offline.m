function cost = compute_cost_offline(p_swarm, model, pos_history, vel_history, u_history)
% compute_cost_offline - gives in output a vector with the values
% of the MPC cost function computed at every time step.

%% Rename parameters

% Swarm
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
    
    % Neighborhood matrix
    M = ones(N,N) - eye(N,N);
    % M = compute_closest_neig(pos, r_comm, max_neig);
    
    sep_cost = zeros(N*(N-1),1);
    dir_cost = zeros(N,1);
    nav_cost = zeros(N,1);
    u_cost = (u_history(step,:))';
    
    pos = (pos_history(step,:))';
    vel = (vel_history(step,:))';
    
    % For every agent define the nonlinear_ls terms
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
            sep_cost((agent-1)*(N-1)+j) = (pos_rel'*pos_rel - d_ref^2);
        end
        
        vel_agent = vel(agent_idx);
        
        % Direction term
        dir_cost(agent) = 1 - (vel_agent'*u_ref)^2/(vel_agent'*vel_agent);
        
        % Navigation term
        nav_cost(agent) = vel_agent'*vel_agent - v_ref^2;
    end
    
    % Assemble expr_y
    cost(step) = (W'*[sep_cost; dir_cost; nav_cost; u_cost].^2);
    
end

end