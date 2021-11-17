%% Create folder for results

date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
dirname = strcat('results/mpc/',date_string);
if ~exist(dirname, 'dir')
    mkdir(dirname)
end

%% Extract trajectories

fontsize = 10;

time_history = (1:1:(k+1))'*dt;
nb_steps_sim = length(time_history)-1;
X_history = x_history';
U_history = u_history';
pos_history = X_history(:,1:3*N);
vel_history = X_history(:,(3*N+1):end);


%% Plot swarm state variables (with SwarmLab)

agents_color = [];
lines_color = [];
plot_state_offline(time_history, pos_history, vel_history, ...
    U_history, agents_color, p_swarm, map, fontsize, lines_color, dirname);


%% Analyse performance

% Compute swarm performance
[safety, order, union, alg_conn, safety_obs, d_min_obs ] = ...
    compute_swarm_performance(pos_history, vel_history, ...
    p_swarm, dirname);

% Plot performance
plot_swarm_performance(time_history, safety, order, ...
    union, alg_conn, safety_obs, d_min_obs, p_swarm, fontsize, dirname);


%% Show solver convergence

if (strcmp(nlp_solver, 'sqp'))
    figure;
    plot(1:nb_steps_sim, sqp_iter, 'r-x');
    xlabel('Iteration','fontsize',fontsize)
    ylabel('Nb SQP iteration','fontsize',fontsize);
    ylim([0 Inf]);
end

fig_handle = figure;
plot(1:nb_steps_sim, time_tot*1e3, 'b-x');
xlabel('Iteration','fontsize',fontsize)
ylabel('Computational time [ms]','fontsize',fontsize);
ylim([0 Inf]);

% Save only if 'dirname' is different from '[]'
if ~isempty(dirname)
    file_path = strcat(dirname,'/sqp_iter');
    savefig(fig_handle,file_path);
    print(fig_handle,file_path,'-dpng','-r300');
end


%% Show residuals

if (strcmp(nlp_solver, 'sqp_rti'))
    
    % Plot figure
    fig_handle = figure;
    plot(1:nb_steps_sim, res(1,:), 'r-x');
    hold on;
    plot(1:nb_steps_sim, res(2,:), 'b-x');
    hold on;
    plot(1:nb_steps_sim, res(3,:), 'g-x');
    legend('sqp iter', 'qp status', 'qp iter');
    
    % Save figure if 'dirname' is different from '[]'
    if ~isempty(dirname)
        file_path = strcat(dirname,'/residuals');
        savefig(fig_handle,file_path);
        print(fig_handle,file_path,'-dpng','-r300');
    end
else

end


%% Show cost convergence

% Plot figure
cost = compute_cost_max_neig_offline(p_swarm, model, pos_history, vel_history, U_history);
fig_handle = figure;
plot(dt*(1:nb_steps_sim), cost);
xlabel('Time [s]','fontsize',fontsize);
ylabel('Cost function residuals','fontsize',fontsize)

% Save figure if 'dirname' is different from '[]'
if ~isempty(dirname)
    file_path = strcat(dirname,'/cost');
    savefig(fig_handle,file_path);
    print(fig_handle,file_path,'-dpng','-r300');
end


%% Save workspace
if ~isempty(dirname)
    file_path = strcat(dirname,'/workspace');
    save(file_path, 'time_history', 'pos_history', 'vel_history', ...
    'U_history', 'p_swarm', 'map');
end


%% Print status

if status == 0
	fprintf('\nsuccess!\n\n');
else
	fprintf('\nsolution failed!\n\n');
end
