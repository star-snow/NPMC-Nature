%% Create the Crazyswarm

crazyswarm = Crazyswarm(is_simulation);
crazyswarm.add_n(p_swarm.nb_agents,swarm_node);
pause(2);


%% Takeoff

height = 1; % meters
duration = 3; % seconds
crazyswarm.takeoff(height, duration, rate);


%% Go to initial positions

duration = 2; % seconds
goal_positions = [p_swarm.Pos0(1,:); -p_swarm.Pos0(2,:); -p_swarm.Pos0(3,:)];
% goal_positions = [0 1; 4 4; 1 1];
crazyswarm.go_to(goal_positions, duration, rate);


%% Closed loop simulation

x_history = [];
u_history = [];

% Set state and input trajectory initialization
x_history(:,1) = x0;
step_mat = repmat((0:1:p),3*N,1);
pos0_traj = repmat(pos0,1,p+1) + v_ref*dt*repmat(u_ref,N,p+1).*step_mat;
x_traj_init = [pos0_traj; ...
    v_ref*repmat(u_ref,N,p+1)];
u_traj_init = zeros(nu, p);

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
    % ocp.set('constr_expr_h', model.expr_h);
    % ocp.set('constr_lh', lh);
    % ocp.set('constr_uh', uh);

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
    % ocp.print('stat');
    res(:,k) = stat(end,:);

    % fprintf('\nstatus = %d, sqp_iter = %d, time_tot = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n', status(k), sqp_iter(k), time_tot(k)*1e3, time_lin(k)*1e3, time_qp_sol(k)*1e3);

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
	x_next = sim.get('xn');
    cmd_positions = reshape(x_next(1:3*N),3,N);
    cmd_positions(2,:) = - cmd_positions(2,:);
    cmd_positions(3,:) = - cmd_positions(3,:);
    % cmd_velocities = reshape(x_next(3*N+1:end),3,N);
    % cmd_velocities(2,:) = - cmd_velocities(2,:);
    % cmd_velocities(3,:) = - cmd_velocities(3,:);
    % cmd_accelerations = reshape(u_history(:,k),3,N);
    
    % Command new state
    crazyswarm.send_pos_commands(cmd_positions, rate);
    % crazyswarm.send_full_commands(cmd_positions, cmd_velocities, cmd_accelerations, rate);

    new_state = crazyswarm.get_states();
    x_history(:,k+1) = new_state * repmat([1 1 -1]',2*p_swarm.nb_agents,1);
    x_history(:,k+1) = x_next;
    
    % Stop simulation when the swarm reaches the goal position
    pos_swarm = x_history(1:3*N,end);
    pos_swarm = reshape(pos_swarm,3,N);
    if sum(pos_swarm(1,:) > 10/10 * map.width) == p_swarm.nb_agents
        break;
    end
end

simulation_time = toc;
disp(strcat('Simulation time: ',num2str(simulation_time)));


%% Land

duration = 0.5; % seconds
crazyswarm.land(duration, rate);


%% Clear ROS related variables and shutdown

% clear crazyswarm
% clear swarm_node
% rosshutdown
