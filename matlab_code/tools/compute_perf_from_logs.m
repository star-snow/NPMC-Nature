function [avg_order, avg_safety, avg_safety_obs, avg_dist_error, ...
    avg_dist_range, avg_speed_error, avg_speed_range, avg_dir_error, ...
    std_order, std_safety, std_safety_obs, std_dist_error, ...
    std_dist_range, std_speed_error, std_speed_range, std_dir_error] = ...
    compute_perf_from_logs(time_history, pos_history, vel_history, t_cut, S, folder)

% Compute performance - This function allows to compute the performance
% from the history of the swarming variables (time, position, velocity,
% parameters).
%
% Inputs:
%   time_history: series of time instants
%   pos_history: series of agents' positions
%   vel_history: series of agents' velocities
%   t_cut: constant transient time to discard in the computation of the
%          performance metrics of distance error/range and speed error/range
%   S : swarm parameters
%   folder : folder
%
% Outputs:
%   avg_order: 
%   avg_safety
%   avg_safety_obs
%   avg_dist_error: normalized average distance error (avg-d_ref)/d_ref
%   avg_dist_range: normalized distance range (max-min)/d_ref
%   avg_speed_error: normalized average speed error (avg-v_ref)/v_ref
%   avg_speed_range: normalized speed range (max-min)/v_ref
%   avg_speed_deltas: 

%% Init variables

[t_steps,nx] = size(pos_history);
t_cut_idx = find(time_history > t_cut,1);
nb_agents = nx/3;
M = zeros(t_steps,nb_agents,nb_agents);
nb_ag_coll = zeros(t_steps,1);
nb_obs_coll = zeros(t_steps,1);

safety = ones(t_steps,1);
order = zeros(t_steps,1);
safety_obs = zeros(t_steps,1);
min_d_obs = zeros(t_steps,nb_agents);
dist_error = zeros(t_steps,1);
dist_range = zeros(t_steps,1);


%% Loop over time

for k = 1:t_steps
    
    %% Safety: reflects the number of collisions among the swarm agents
    
    pos_k = pos_history(k,:);
    pos_k = reshape(pos_k,3,[]);
    dist_vect_k = pdist(pos_k');
    if ~isempty(dist_vect_k)
        nb_ag_coll(k) = sum(dist_vect_k < 2*S.r_coll);
        safety(k) = 1 - (sum(nb_ag_coll(k)) / length(dist_vect_k));
    else
        nb_ag_coll(k) = 0;
        safety(k) = 1;
    end
    
    
    %% Order: reflects the correlation of the velocity vectors
    distance_matrix_k = squareform(dist_vect_k);
    M(k,:,:) =  compute_neighborhood(distance_matrix_k, S.r, S.max_neig);
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
    if S.is_active_cyl
        
        pos_k = pos_history(k,:);
        pos_k = reshape(pos_k,3,[]);
        
        c_cyl = S.cylinders(1:2,:);
        r_cyl = S.cylinders(3,:);
        
        D_cyl = pdist2(pos_k(1:2,:)',c_cyl');
        min_d_obs(k,:) = min(pdist2(pos_k(1:2,:)',c_cyl') - repmat(r_cyl, nb_agents, 1),[],2);
        nb_obs_coll(k) = nb_obs_coll(k) + sum(sum(D_cyl < repmat(r_cyl, nb_agents, 1)));
        nb_possible_coll = nb_possible_coll + nb_agents*length(r_cyl);
        
        safety_obs(k) = 1 - (nb_obs_coll(k)/nb_possible_coll);
    end
    
    %% Normalized distance error and range
    
    if time_history(k) > t_cut
        max_dist_k = 0;
        min_dist_k = 1000000;
        for agent = 1:nb_agents
            neig = (M(k,:,agent)==true);
            nn = sum(neig);
            if nn ~= 0
                dist_k_agent = abs(distance_matrix_k(agent, neig) - S.d) / S.d;
                dist_error(k) = dist_error(k) + sum(dist_k_agent)/nn;
                if max_dist_k < max(distance_matrix_k(agent, neig))
                    max_dist_k = max(distance_matrix_k(agent, neig));
                end
                if min_dist_k > min(distance_matrix_k(agent, neig))
                    min_dist_k = min(distance_matrix_k(agent, neig));
                end
            end
        end
        dist_error(k) = dist_error(k)/nb_agents;
        dist_range(k) = (max_dist_k - min_dist_k)/ S.d;
        
    end
    
end


%% Normalized speed error and range

vels = reshape(vel_history',3,[]);
speeds = sqrt(sum(vels.^2));
speeds_r = reshape(speeds, S.nb_agents, []);
speed_errors = abs( speeds - S.v_swarm)/S.v_swarm;
speed_ranges = (max(speeds_r) - min(speeds_r))/S.v_swarm;
speed_errors_r = reshape(speed_errors, S.nb_agents, []);
speed_error = sum(speed_errors_r)/S.nb_agents;
speed_ranges(1:t_cut) = 0;
speed_error(1:t_cut) = 0;

%% Direction error

dir_errors = (1 - (vels' * S.u_migration)./speeds')/2;
dir_errors_r = reshape(dir_errors, S.nb_agents, []);
dir_error = sum(dir_errors_r)/S.nb_agents;

if ~isempty(find(safety~=1))
    disp(strcat("Detected collision(s) in ",folder));
end

%% Aggreate values

avg_order = sum(order,'omitnan')/length(order);
avg_safety = sum(safety,'omitnan')/length(safety);
avg_safety_obs = sum(safety_obs,'omitnan')/length(safety_obs);
avg_dist_error = sum(dist_error(t_cut_idx:end),'omitnan')/length(dist_error(t_cut_idx:end));
avg_dist_range = sum(dist_range(t_cut_idx:end),'omitnan')/length(dist_range(t_cut_idx:end));
avg_speed_error = sum(speed_error(t_cut_idx:end),'omitnan')/length(speed_error(t_cut_idx:end));
avg_speed_range = sum(speed_ranges(t_cut_idx:end),'omitnan')/length(speed_ranges(t_cut_idx:end));
avg_dir_error = sum(dir_error,'omitnan')/length(dir_error);

std_order = std(order,'omitnan');
std_safety = std(safety,'omitnan');
std_safety_obs = std(safety_obs,'omitnan');
std_dist_error = std(dist_error(t_cut_idx:end),'omitnan');
std_dist_range = std(dist_range(t_cut_idx:end),'omitnan');
std_speed_error = std(speed_error(t_cut_idx:end),'omitnan');
std_speed_range = std(speed_ranges(t_cut_idx:end),'omitnan');
std_dir_error = std(dir_error,'omitnan');


end

