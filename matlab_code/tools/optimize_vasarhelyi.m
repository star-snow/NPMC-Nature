%% Genetic Algorithm for Vasarhelyi flocking
% This script applies the genetic algorithms theory to find the optimal
% parameters for the Vasarhelyi's algorithm.
% GA searches for a maximum of the performance function, a real 
% valued function of the Vasarhelyi's parameters.


%% Plot objective
% plotobjective(@fcn, [0  0  0; 10  10  10]);


%% Parameters
% At least two input arguments:
%       1) a fitness function 
%       2) the number of variables in the problem. 
% Output arguments:
%       1) |x|, the best point found.
%       2) |Fval|, the function value at the best point.
%       3) |exitFlag|, the reason why |ga| stopped. 
%       4) |Output|, information about the performance of the solver.

% x = [ S.p_rep; S.r0_fric; S.C_fric; S.v_fric; S.p_fric; S.a_fric; ...
%     S.r0_shill; S.v_shill; S.p_shill; S.a_shill];

results_path = 'results/vasarhelyi/optimization_1/';

W_perf = [0.1 1 1];
v_ref = 0.5;
d_ref = 0.8;
map_size = 8;
nb_blocks = 3;
building_width = 0.7;

FitnessFunction     = @(x) params2fitness(x, d_ref, v_ref, map_size, ...
    nb_blocks, building_width, W_perf, results_path);
numberOfVariables   = 10;


%% Adding Visualization
% 1) |gaplotbestf|, best and mean score of the population at every generation. 
% 2) |gaplotstopping|, percentage of stopping criteria satisfied.

opts = optimoptions(@ga,'PlotFcn',{@gaplotbestf,@gaplotstopping});


%% Specifying Population Options
% The default initial population is created using a uniform random number
% generator. Default values for the population size and the range of the
% initial population are used to create the initial population.
%
%%%%%%%%%%%%%%  Size  %%%%%%%%%%%%%%
% Default:  50      when the number of decision variables < 5 
%           200     otherwise
% If the nb of variables is small, we can reduce the size. If it is big, we
% can increase it.
%
%%%%%%%%%%%%%%  Range  %%%%%%%%%%%%%%
% Default: uniform random number in the range 0 to 1.
% If the range has one column, the range of every variable is the given range.
% To specify a different initial range for each variable, the range must be 
% specified as a matrix with two rows and |numberOfVariables| columns. 

% opts = optimoptions(@ga,'PopulationSize',2);
opts.PopulationSize = 15;
opts.InitialPopulationRange = [  0.1  5   0.04  0.05  2.5  0.05  0.1  0.8  1.5  1;  % lower bound
                                 0.3  10  0.06  0.07  3.5  0.1   0.2  1    3    2]; % upper bound

                             
%% Modifying the Stopping Criteria
% 4 different criteria:
%       1) maximum number of generations is reached (default is 100). 
%       2) no change in the best fitness value for some time.
%       3) no change in the best fitness value for some generations.
%       4) maximum time limit in seconds

opts = optimoptions(opts, 'MaxGenerations', 60, 'MaxStallGenerations', 6);


%% Choosing GA operators
% Operators produce the next generation of the population:
%       1) scaling,
%       2) selection, 
%       3) crossover, 
%       4) mutation.
%
% The best function value can improve or it can get worse by choosing
% different operators. Choosing a good set of operators for your problem is
% often best done by experimentation.
%
% Here we choose |fitscalingprop| for |FitnessScalingFcn| and
% |selectiontournament| for |SelectionFcn|.
opts = optimoptions(opts, 'SelectionFcn', @selectiontournament, ...
                        'FitnessScalingFcn', @fitscalingprop);

                    
%% Reproducing your results
% By default, GA starts with a random initial population which is
% created using MATLAB random number generators. 
% Every time a random number is generated, the state of the random number 
% generators change. This means that even if you do not change any options, 
% when you run again you can get different results.
% If you want to reproduce your results before you run |ga|, you can save 
% the state of the random number stream.

% thestate = rng;


%% Run the GA solver

lb = [0.1  5   0.04  0.5  2.5  0.05  0.1  0.8  1.5  1];
ub = [0.3  10  0.06  0.7  3.5  0.1   0.2  1    3    2];

[x, Fval, exitFlag, Output] = ga(FitnessFunction, numberOfVariables, ...
    [], [], [], [], lb, ub, [], [], opts);

p_rep       = x(1);
r0_fric     = x(2);
C_fric      = x(3);
v_fric      = x(4);
p_fric      = x(5);
a_fric      = x(6);
r0_shill    = x(7);
v_shill     = x(8);
p_shill     = x(9);
a_shill     = x(10);

save(strcat(results_path,'optim_result'),'x', 'Fval', 'exitFlag', 'Output');

