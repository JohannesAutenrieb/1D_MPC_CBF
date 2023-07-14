%*************************************************************************
%
% Filename:				Main.m
%
% Authors:				Johannes Autenrieb, johannes.autenrieb@outlook.com
% Created:				27-Nov-2022
%
%*************************************************************************
%
% Description:
%		Simple first order control exampl for a study on the application of 
%       CBF in the control of linear systems based on the paper of 
%
% Input parameter:
%		- none
%		- none
%
% Output parameter:
%		- none
%		- none
%
%% #######################    SCRIPT START   ############################

%% Pre-steps
clear workspace
clear all; close all;
addpath(genpath('./../../include'))

UNIT_RAD2DEG = 180/pi;
UNIT_DEG2RAD = 1 / UNIT_RAD2DEG ; 

%% init all relevant simulation settings and simulation cases
sim.dt = 0.01;
sim.sim_t = 10;


%% init all relevant parameter of the regarded case and model
params.ap = -1;
params.bp = 1;
params.k_fb = -1;
params.a_d = (params.ap+params.bp*params.k_fb);
params.x_d  = -1 ;
params.k_r = -params.a_d/params.bp;
params.gamma = 20;
params.orderOfAlpha =1;
params.u_0 = 0.8;
params.x_max = 0.3;
params.x_min = -0.3;


%% init state of system
init.xp_0 = 0;

%% initialize and pre-allocate run time variables.
total_k = ceil(sim.sim_t / sim.dt);
x0 = [init.xp_0]';
x =x0;
t = 0; 
t_comp = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Simulation with Saftey filter constrained control input (QP)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:total_k

    xp      = x; 
    
    % Computing needed control input to get desired cruise behaviour
    tStart = tic;
    u_des   = params.k_fb*xp + params.k_r*params.x_d;
    
    % controller and saftey filter on computed control input
    u       = SafetyFilter(x,u_des, params);
    tDurPointwise = toc(tStart) ;
    
    % Run one time step propagation.
    [ts_temp, xs_temp] = ode45(@(t, s) Dynamics(x,u,params), [t t+sim.dt], x);
 
    % saving the state of current time step
    x = xs_temp(end, :)';
    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    
    % Saving control relevant information of current time step
    us(k, :) = u';
    
    % Setting time for next time step
    t = t + sim.dt;
    
    % saving comp. time of MPC
    t_comp = [t_comp tDurPointwise];
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%% Plot results
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_p = xs; 
innput = us;

subplot(3,1,1);
p = plot(ts, state_p,'b', 'LineWidth',1.5);
hold on;
p2 = plot(ts, params.x_max*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
p2 = plot(ts, params.x_min*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
p3 = plot(ts, params.x_d*ones(size(ts,2,1)),'--k', 'LineWidth',1.5);
legend([p p2 p3], 'Closeed-Loop w/ CBF-QP', 'State Constraint - x_{max/min}','Desired State - x_{des}', 'Location', 'Southeast');
set(gca,'FontSize',14);
ylabel("State x_p (m)");
xlabel("Time t (s)");

subplot(3,1,2);
p = plot(ts(1:end-1), us,'b', 'LineWidth',1.5);
hold on;
p2 = plot(ts, params.u_0*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
p2 = plot(ts, -params.u_0*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
legend([p p2], 'Closeed-Loop w/ CBF-QP','Input Constraints', 'Location', 'Northeast');
set(gca,'FontSize',14); 
ylabel("Control Input u (m/s)");
xlabel("Time t (s)");

subplot(3,1,3);
p = stem((1:1:total_k),t_comp);
legend(p, 'MPC w/ CBF constrains', 'Location', 'Northeast');
ylabel("Computation time (s)");
xlabel("Time step k");
set(gca,'FontSize',14);  
   

% #######################     SCRIPT END    ############################