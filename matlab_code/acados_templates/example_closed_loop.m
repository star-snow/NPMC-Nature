close all
clear all

% Add project to path
addpath(genpath('/home/esoria/Developer/sp2018_uavSim'));

%% Test of native matlab interface

% Check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('ERROR: env.sh has not been sourced! Before executing this example, run:');
	disp('source env.sh');
	return;
end

date_string = datestr(now,'yyyy_mm_dd_HH:MM:SS');
dirname = strcat('results/mpc/',date_string);
if ~exist(dirname, 'dir')
    mkdir(dirname)
end


%% Arguments

map = [];

% Import map param, in map structure
run('./params/param_map.m');
% Import swarming param, in s_param structure
run('./params/param_swarm.m');

% Overwrite and add some param
% p_swarm.nb_agents = 5;
% p_swarm.max_neig = 2;
p_swarm.d_ref = 0.5;

% Rename param
N = p_swarm.nb_agents; % nb of agents
max_neig = p_swarm.max_neig; % number of neighbours
v_ref = p_swarm.v_ref;
u_ref = p_swarm.u_ref;
d_ref = p_swarm.d_ref;
max_a = sqrt(p_swarm.max_a^2/3);
r_coll = p_swarm.r_coll;

% Time param for MPC
dt = 0.1; % sample time [s]
T_p = 4; % prediction horizon [s]
p = floor(T_p/dt); % nb of steps over the prediction horizon
T = 15; % time of the whole simulation [S]
nb_steps_sim = floor(T/dt); % nb of steps over the whole simulation

if 1
	compile_interface = 'auto';
	codgen_model = 'true';
	gnsf_detect_struct = 'true';
else
	compile_interface = 'auto';
	codgen_model = 'false';
	gnsf_detect_struct = 'false';
end

% Simulation
sim_method = 'erk'; % erk, irk, irk_gnsf
sim_sens_forw = 'false'; % true, false
sim_num_stages = 4;
sim_num_steps = 3;

% OCP
param_scheme = 'multiple_shooting_unif_grid';
nlp_solver = 'sqp'; % sqp, sqp_rti
nlp_solver_exact_hessian = 'false';
regularize_method = 'no_regularize'; % no_regularize, project,...
	% project_reduc_hess, mirror, convexify
nlp_solver_max_iter = 4;
nlp_solver_tol_stat = 1;
nlp_solver_tol_eq   = 1e-1;
nlp_solver_tol_ineq = 1e-2;
nlp_solver_tol_comp = 1e-1;
nlp_solver_step_length = 0.05;
nlp_solver_ext_qp_res = 1; % with 10 nothing changes
qp_solver = 'partial_condensing_hpipm';
        % full_condensing_hpipm, partial_condensing_hpipm
qp_solver_iter_max = 20;
qp_solver_cond_N = p/2;
qp_solver_warm_start = 0;
qp_solver_cond_ric_alg = 0; % 0: dont factorize hessian in the condensing; 1: factorize
qp_solver_ric_alg = 0; % HPIPM specific
ocp_sim_method = 'irk_gnsf'; % erk, irk, irk_gnsf
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 3;
cost_type = 'nonlinear_ls'; % linear_ls, ext_cost

model_name = 'cl_swarming';

%% Model

model = swarming_model(p_swarm);

% Dimensions
nx = model.nx;
nu = model.nu;
ny = model.ny; % number of outputs in lagrange term
ny_e = model.ny_e; % number of outputs in mayer term

nbx = 0;
nbu = 0;
ng = 0;
ng_e = 0;
% nh = nu;
% nh_e = 0;

% Cost
W = diag(model.W); % weight matrix in lagrange term
W_e = diag(model.W_e); % weight matrix in mayer term

y_ref = zeros(ny, 1); % output reference in lagrange term
y_ref_e = zeros(ny_e,1); % output reference in mayer term

% Constraints
x0 = [p_swarm.Pos0(:); p_swarm.Vel0(:)];
pos0 = p_swarm.Pos0(:);

% pos0 = 10*rand(3*N,1);
% vel0 = repmat([6;0;0],N,1) + 0.05*rand(3*N,1);
% x0 = [pos0; vel0];

%% Acados ocp model

ocp_model = acados_ocp_model();
ocp_model.set('name', model_name);
ocp_model.set('T', T_p);

% Dimensions
ocp_model.set('dim_nx', nx);
ocp_model.set('dim_nu', nu);

if strcmp(cost_type, 'nonlinear_ls')
	ocp_model.set('dim_ny', ny);
	ocp_model.set('dim_ny_e', ny_e);
end

ocp_model.set('dim_nh', model.nh);
ocp_model.set('dim_nh_e', model.nh_e);

% Symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

% Cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);

if strcmp(cost_type, 'nonlinear_ls')
	ocp_model.set('cost_expr_y', model.expr_y);
	ocp_model.set('cost_expr_y_e', model.expr_y_e);
	ocp_model.set('cost_W', W);
	ocp_model.set('cost_W_e', W_e);
	ocp_model.set('cost_y_ref', y_ref);
	ocp_model.set('cost_y_ref_e', y_ref_e);
else % ext_cost
	ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
	ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost);
end

% Dynamics
if (strcmp(ocp_sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('dyn_expr_f', model.expr_f_expl);
elseif (strcmp(ocp_sim_method, 'irk') | strcmp(ocp_sim_method, 'irk_gnsf'))
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('dyn_expr_f', model.expr_f_impl);
else
	ocp_model.set('dyn_type', 'discrete');
	ocp_model.set('dyn_expr_phi', model.expr_phi);
end

% Constraints
ocp_model.set('constr_x0', x0);

ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', model.lh);
ocp_model.set('constr_uh', model.uh);
% ocp_model.set('constr_expr_h_e', model.expr_h_e);
% ocp_model.set('constr_lh_e', model.lh_e);
% ocp_model.set('constr_uh_e', model.uh_e);

ocp_model.model_struct

%% Acados ocp options

ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme', param_scheme);
ocp_opts.set('param_scheme_N', p);
if (strcmp(param_scheme, 'multiple_shooting'))
	ocp_opts.set('param_scheme_shooting_nodes', shooting_nodes);
end
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_opts.set('regularize_method', regularize_method);
if (strcmp(nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
	ocp_opts.set('nlp_solver_tol_stat', nlp_solver_tol_stat);
	ocp_opts.set('nlp_solver_tol_eq', nlp_solver_tol_eq);
	ocp_opts.set('nlp_solver_tol_ineq', nlp_solver_tol_ineq);
	ocp_opts.set('nlp_solver_tol_comp', nlp_solver_tol_comp);
    ocp_opts.set('nlp_solver_step_length', nlp_solver_step_length);
end
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
	ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
end
ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
if (strcmp(ocp_sim_method, 'irk_gnsf'))
	ocp_opts.set('gnsf_detect_struct', gnsf_detect_struct);
end

ocp_opts.opts_struct

%% Acados ocp

% Create ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp
ocp.C_ocp
ocp.C_ocp_ext_fun
%ocp.model_struct

%% Acados simulation model

sim_model = acados_sim_model();

% Dimentions
sim_model.set('dim_nx', nx);
sim_model.set('dim_nu', nu);

% Symbolics
sim_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	sim_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	sim_model.set('sym_xdot', model.sym_xdot);
end
% model
sim_model.set('T', dt);
if (strcmp(sim_method, 'erk'))
	sim_model.set('dyn_type', 'explicit');
	sim_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
	sim_model.set('dyn_type', 'implicit');
	sim_model.set('dyn_expr_f', model.expr_f_impl);
end

%sim_model.model_struct



%% Acados simulation options
sim_opts = acados_sim_opts();
sim_opts.set('compile_interface', compile_interface);
sim_opts.set('codgen_model', codgen_model);
sim_opts.set('num_stages', sim_num_stages);
sim_opts.set('num_steps', sim_num_steps);
sim_opts.set('method', sim_method);
sim_opts.set('sens_forw', sim_sens_forw);
if (strcmp(sim_method, 'irk_gnsf'))
	sim_opts.set('gnsf_detect_struct', gnsf_detect_struct);
end

%sim_opts.opts_struct



%% Acados simulation

% Create sim
sim = acados_sim(sim_model, sim_opts);


%% Closed loop simulation

x_history = [];
x_history(:,1) = x0;
u_history = [];

% Set state and input trajectory initialization
step_mat = repmat((0:1:p),3*N,1);
pos0_traj = repmat(pos0,1,p+1) + v_ref*dt*repmat(u_ref,N,p+1).*step_mat;
x_traj_init = [pos0_traj; ...
    v_ref*repmat(u_ref,N,p+1)];
u_traj_init = zeros(nu, p);

% Initialize variables
% status = zeros(1, nb_steps_sim);
% sqp_iter = zeros(1, nb_steps_sim);
% time_tot = zeros(1, nb_steps_sim);
% time_lin = zeros(1, nb_steps_sim);
% time_qp_sol = zeros(1, nb_steps_sim);

status = [];
sqp_iter = [];
time_tot = [];
time_lin = [];
time_qp_sol = [];

if (strcmp(nlp_solver, 'sqp'))
    res = zeros(7, nb_steps_sim);
else
    res = zeros(3, nb_steps_sim);
end

tic;

for k = 1:nb_steps_sim

	% Set initial condition x0
	ocp.set('constr_x0', x_history(:,k));
%     ocp.set('constr_expr_h', model.expr_h);
%     ocp.set('constr_lh', lh);
%     ocp.set('constr_uh', uh);

	% Set trajectory initialization (if not, set internally using previous solution)
	ocp.set('init_x', x_traj_init);
	ocp.set('init_u', u_traj_init);

	% solve OCP
	ocp.solve();

    status(k) = ocp.get('status');
    sqp_iter(k) = ocp.get('sqp_iter');
    time_tot(k) = ocp.get('time_tot');
    time_lin(k) = ocp.get('time_lin');
    time_qp_sol(k) = ocp.get('time_qp_sol');
    stat = ocp.get('stat');
%     ocp.print('stat');
    res(:,k) = stat(end,:);

    fprintf('\nstatus = %d, sqp_iter = %d, time_tot = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n', status(k), sqp_iter(k), time_tot(k)*1e3, time_lin(k)*1e3, time_qp_sol(k)*1e3);

	% Get solution for initialization of next NLP
	x_traj = ocp.get('x');
	u_traj = ocp.get('u');
    
	% Shift trajectory for initialization
	x_traj_init = [x_traj(:,2:end), x_traj(:,end)];
	u_traj_init = [u_traj(:,2:end), u_traj(:,end)];

	% Get solution for simulation
	u_history(:,k) = ocp.get('u', 0);

	% Set initial state of simulation
	sim.set('x', x_history(:,k));
	% set input in sim
	sim.set('u', u_history(:,k));

	% Simulate state
	sim.solve();

	% Get new state
	x_history(:,k+1) = sim.get('xn');

end

simulation_time = toc;
disp(strcat('Simulation time: ',num2str(simulation_time)));

%% Extract trajectories

fontsize = 12;

% linspace(0,T,nb_steps_sim+1)';
time_history = (1:1:(k+1))'*dt;
nb_steps_sim = length(time_history)-1;
x_history = x_history';
u_history = u_history';
pos_history = x_history(:,1:3*N);
vel_history = x_history(:,(3*N+1):end);

%% Plots with swarming functions

% Plot state variables
agents_color = [];
lines_color = [];
plot_state_offline(time_history, pos_history, vel_history, ...
    u_history, agents_color, p_swarm, map, fontsize, lines_color, dirname);

%% Analyse performance

% Compute swarm performance
[safety, order, union, alg_conn, safety_obs ] = ...
    compute_swarm_performance(pos_history, vel_history, ...
    p_swarm, dirname);

% Plot performance
plot_swarm_performance(time_history, safety, order, ...
    union, alg_conn, safety_obs, p_swarm, fontsize, dirname);

%% Plots without swarming functions

% % Plot trajectories of the agents
% figure;
% for agent = 1:N
%     hold on;
%     plot3(pos_history(:,(agent-1)*3+1), pos_history(:,(agent-1)*3+2), ...
%         - pos_history(:,(agent-1)*3+3));
% end
% % title('Agents trajectories');
% xlabel('X Position [m]','fontsize',fontsize);
% ylabel('Y Position [m]','fontsize',fontsize);
% zlabel('Z Position [m]','fontsize',fontsize);
% view(2);
% 
% figure;
% plot(time_history(1:(end-1)), u_history);
% xlim([0 time_history(end-1)]);
% xlabel('Time [s]','fontsize',fontsize);
% ylabel('Control inputs [m/s^2]','fontsize',fontsize);

%% Show solver convergence

if (strcmp(nlp_solver, 'sqp'))
    figure;
    plot(1:nb_steps_sim, sqp_iter, 'r-x');
    xlabel('Iteration','fontsize',fontsize)
    ylabel('Nb SQP iteration','fontsize',fontsize);
    ylim([0 Inf]);
end

figure;
plot(1:nb_steps_sim, time_tot*1e3, 'b-x');
xlabel('Iteration','fontsize',fontsize)
ylabel('Simulation time [ms]','fontsize',fontsize);
ylim([0 Inf]);

%% Show residuals
if (strcmp(nlp_solver, 'sqp_rti'))
    figure;
    plot(1:nb_steps_sim, res(1,:), 'r-x');
    hold on;
    plot(1:nb_steps_sim, res(2,:), 'b-x');
    hold on;
    plot(1:nb_steps_sim, res(3,:), 'g-x');
    legend('sqp iter', 'qp status', 'qp iter');
else

end

%% Show cost convergence

cost = compute_cost_offline(p_swarm, model, pos_history, vel_history, u_history);
figure;
plot(dt*(1:nb_steps_sim), cost);
xlabel('Time [s]','fontsize',fontsize);
ylabel('Cost function residuals','fontsize',fontsize)

if status == 0
	fprintf('\nsuccess!\n\n');
else
	fprintf('\nsolution failed!\n\n');
end
