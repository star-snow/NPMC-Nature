%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vicsek Paramteres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% Repulsion %%%%%%%%%%%%%%%%%%%%%%
%
% Repulsion range
p_swarm.p_rep = 0.29; % 0.13;

%%%%%%%%%%%%%%%%%% Friction %%%%%%%%%%%%%%%%%%%%%%
%
% Stopping point offset of alignment
p_swarm.r0_fric = 6.98; %85.3
% Coefficient of velocity alignment
p_swarm.C_fric = 0.06;
% Velocity slack of alignement
p_swarm.v_fric = 0.63;
% Gain of braking curve
p_swarm.p_fric = 3.34;
% Acceleration of braking curve
p_swarm.a_fric = 0.05; %4.16;

%%%%%%%%%%%%%%%%%% Obstacles and wall %%%%%%%%%%%%
%
% Stopping point offset of walls
p_swarm.r0_shill = 0.10; % 0.3;
% Velocity of virtual shill agents
p_swarm.v_shill = 0.81; % 13.6;
% Gain of braking curve for walls
p_swarm.p_shill = 2.99; % 3.55;
% Acceleration of braking curve for walls
p_swarm.a_shill = 1.17; % 3.02
