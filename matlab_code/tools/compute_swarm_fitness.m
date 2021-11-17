function [order, safety, safety_obs, min_d_obs, dist_error, speed_error, ...
    dir_error, speed_delta ] = ...
    compute_swarm_fitness(pos_history, vel_history, accel_history, p_swarm, dirname)
% Compute performance - This function allows to compute the performance
% from the history of the swarming variables (time, position, velocity,
% acceleration).
%
% Inputs:
%   time_history: series of time steps
%   pos_history: series of agents' positions
%   vel_history: series of agents' velocities
%   u_history: series of agents' accelerations
%
% Outputs:
%   order
%   safety
%   safety_obs
%   min_d_obs
%   dist_error: average normalized distance error
%   speed_error
%   speed_deltas: 

%% Init variables

[t_steps,nx] = size(pos_history);
nb_agents = nx/3;
M = zeros(t_steps,nb_agents,nb_agents);
nb_ag_coll = zeros(t_steps,1);
nb_obs_coll = zeros(t_steps,1);

safety = ones(t_steps,1);
order = zeros(t_steps,1);
safety_obs = zeros(t_steps,1);
min_d_obs = zeros(t_steps,nb_agents);
dist_error = zeros(t_steps,1);

%% Loop over time

for k = 1:t_steps
    
    %% Safety: reflects the number of collisions among the swarm agents
    
    pos_k = pos_history(k,:);
    pos_k = reshape(pos_k,3,[]);
    dist_vect_k = pdist(pos_k');
    if ~isempty(dist_vect_k)
        nb_ag_coll(k) = sum(dist_vect_k < 2*p_swarm.r_coll);
        safety(k) = 1 - (sum(nb_ag_coll(k)) / length(dist_vect_k));
    else
        nb_ag_coll(k) = 0;
        safety(k) = 1;
    end
    
    %% Order: reflects the correlation of the velocity vectors
    distance_matrix_k = squareform(dist_vect_k);
    M(k,:,:) =  compute_neighborhood(distance_matrix_k, p_swarm.r, p_swarm.max_neig);
    for agent = 1:nb_agents
        neig = (M(k,:,agent)==true);
        nn = sum(neig);
        vel_k = vel_history(k,:);
        vel_k = reshape(vel_k,3,[]);
        if nn ~= 0
            scalar_prod = vel_k(:, agent)' * vel_k(:, neig);
            speed_agent = norm(vel_k(:, agent));
            speed_neig = sqrt(sum((vel_k(:, neig) .^ 2), 1));
            order(k) = order(k) + ...
                sum (scalar_prod ./ (speed_agent * speed_neig)) / nn;
        else
            order(k) = order(k) + 1;
        end
    end
    order(k) = order(k)/nb_agents;
    
    %% Safety with obstacles: reflects the number of collisions between swarm agents and obstacles
    
    nb_possible_coll = 0;
    if p_swarm.is_active_cyl
        
        pos_k = pos_history(k,:);
        pos_k = reshape(pos_k,3,[]);
        
        if p_swarm.is_active_spheres
            c_spheres = p_swarm.spheres(1:3,:);
            r_spheres = p_swarm.spheres(4,:);
            
            D_spheres = pdist2(pos_k',c_spheres');
            nb_obs_coll(k) = nb_obs_coll(k) + sum(sum(D_spheres < repmat(r_spheres, nb_agents, 1)));
            nb_possible_coll = nb_possible_coll + nb_agents*length(r_spheres);
        end
        if p_swarm.is_active_cyl
            c_cyl = p_swarm.cylinders(1:2,:);
            r_cyl = p_swarm.cylinders(3,:);
            
            D_cyl = pdist2(pos_k(1:2,:)',c_cyl');
            min_d_obs(k,:) = min(pdist2(pos_k(1:2,:)',c_cyl') - repmat(r_cyl, nb_agents, 1),[],2); 
            nb_obs_coll(k) = nb_obs_coll(k) + sum(sum(D_cyl < repmat(r_cyl, nb_agents, 1)));
            nb_possible_coll = nb_possible_coll + nb_agents*length(r_cyl);
        end
        if p_swarm.is_active_arena
            x_coll = sum(pos_k(1,:)< p_swarm.x_arena(1,1) | pos_k(1,:)> p_swarm.x_arena(1,2));
            y_coll = sum(pos_k(2,:)< p_swarm.x_arena(2,1) | pos_k(2,:)> p_swarm.x_arena(2,2));
            z_coll = sum(-pos_k(3,:)< p_swarm.x_arena(3,1) | -pos_k(3,:)> p_swarm.x_arena(3,2));
            nb_obs_coll(k) = nb_obs_coll(k) + x_coll + y_coll + z_coll;
            nb_possible_coll = nb_possible_coll + nb_agents;
        end
        
        safety_obs(k) = 1 - (nb_obs_coll(k)/nb_possible_coll);
    end
    

    %% Normalized distance error

    for agent = 1:nb_agents
        neig = (M(k,:,agent)==true);
        nn = sum(neig);
        if nn ~= 0
            dist_k_agent = abs(distance_matrix_k(agent, neig) - p_swarm.d_ref) / p_swarm.d_ref;
            dist_error(k) = dist_error(k) + sum(dist_k_agent)/nn;
        end
    end
    dist_error(k) = dist_error(k)/nb_agents;


end

%% Normalized speed errors

vels = reshape(vel_history',3,[]);
speeds = sqrt(sum(vels.^2));
speed_errors = abs( speeds - p_swarm.v_ref)/p_swarm.v_ref;
speed_errors_r = reshape(speed_errors, p_swarm.nb_agents, []);
speed_error = sum(speed_errors_r)/p_swarm.nb_agents;

%% Direction error

dir_errors = (1 - (vels' * p_swarm.u_ref)./speeds')/2;
dir_errors_r = reshape(dir_errors, p_swarm.nb_agents, []);
dir_error = sum(dir_errors_r)/p_swarm.nb_agents;

%% Compute input deltas

accel = accel_history;
accel_r = reshape(accel',3,[]);
speed_deltas = sqrt(sum(accel_r.^2))/p_swarm.max_a;
speed_deltas_r = reshape(speed_deltas, p_swarm.nb_agents, []);
speed_delta = sum(speed_deltas_r)/p_swarm.nb_agents;


%% Save workspace

if ~isempty(dirname)
    path = strcat(dirname,'/fitness');
    save(path);
end

end

