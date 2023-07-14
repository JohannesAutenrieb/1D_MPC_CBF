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
%		Simple 1d control example for a study on the application of 
%       CBF in MPC
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
sim.dt = 0.1;
sim.sim_t = 5;
sim.PlotDuringSimulation = 0;


%% init all relevant parameter of the regarded case and model
params.ap = -1;
params.bp = 1;
params.k_fb = -1;
params.a_des = (params.ap+params.bp*params.k_fb);
params.k_r = params.a_des/params.bp;
params.x_d = -2;
params.gamma = 0.1;
params.orderOfAlpha =1;
params.u_max = 1.5;
params.u_min = -1.5;
params.x_max = 0.3;
params.x_min = -0.3;

%% storage pre-allocation
total_k = ceil(sim.sim_t / sim.dt);

%% init state of system
init.xp_0 = 0;
init.u_0 = 0;

%% initialize and pre-allocate run time variables.
x0 = [init.xp_0]';
x = digits(3);
x =x0;
t = 0;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Definition of MPC with CBF constraints
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Defintion of MPC relevant parameter and information

nx = 1; % Number of states
nu = 1; % Number of inputs

% LQ-like Weights for cost function
Q = 1.2;     
R = 0.8;

% prediction horizon (predicted number of time steps)
N = 2;

% defining set point variables
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
x_d = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
pastu=sdpvar(1);

constraints = [-.1 <= diff([pastu u{:}]) <= .1];

objective = 0;
for k = 1:N
    objective = objective + (x{k}-x_d{k})'*Q*(x{k}-x_d{k}) + (u{k})'*R*(u{k});
    constraints = [constraints, x{k+1} == params.ap*x{k}+params.bp*u{k}];
    constraints = [constraints, params.u_min <= u{k}<= params.u_max];
    constraints = [constraints; (params.x_max - x{k+1}) - (params.x_max - x{k}) + params.gamma * (params.x_max - x{k}) >= 0]; %CBF const.: ∆h + gamma*h  
    constraints = [constraints; (x{k+1}-params.x_min) - (x{k}-params.x_min) + params.gamma * (x{k} - params.x_min) >= 0]; %CBF const.: ∆h + gamma*h
end
objective = objective + (x{N+1}-x_d{N+1})'*(x{N+1}-x_d{N+1});

parameters_in = {x{1},[x_d{:}],pastu};
solutions_out = {[u{:}], [x{:}]};

controller = optimizer(constraints, objective,sdpsettings('solver','IPOPT','verbose',0),parameters_in,solutions_out);

x = init.xp_0;
u = init.u_0;
ts(1) = 0;
clf;
hold on
xhist = x;
uhist = u;
xs_smooth = x;
x_ode_tp = 0;
t_comp = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Simulation with Saftey filter constrained control input (QP)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:total_k

    % desired state as time series over prediction horizon
    x_d_time = params.x_d*ones(1,N+1);  
    
    % MPC comp. procedure 
    inputs = {x,x_d_time ,u};
    
    tStart = tic;
    [solutions,diagnostics] = controller{inputs};
    tDurMPC = toc(tStart) ;
    
    U = solutions{1};
    u = U(1);
    X = solutions{2};
    
    if diagnostics == 1
        error('The problem is infeasible');
    end    
    
    if sim.PlotDuringSimulation == 1
        subplot(1,2,1);stairs(i:i+length(U)-1,U,'r')
        subplot(1,2,2);cla;stairs(i:i+N,X(1,:),'b');hold on;stairs(i:i+N,future_r(1,:),'k')
        stairs(1:i,xhist(1,:),'g') 
    end 
    

    %Run one time step propagation.
    xs_temp = params.ap*x+params.bp*U(1);
    t = t+sim.dt;
    x = xs_temp(end, :);
    pause(0.05)  

    % saving the state of current time step
    ts(k+1) = t;
     
    % Saving state & control relevant information of current time step
    xhist = [xhist x];
    uhist = [uhist u];
    
    % saving comp. time of MPC
    t_comp = [t_comp tDurMPC];
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%% Plot results
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_p = xhist; 
innput = uhist;
comp_time = t_comp;

subplot(3,1,1);
p = plot((0:sim.dt:sim.sim_t), state_p,'b', 'LineWidth',1.5);
hold on;
p2 = plot(ts, params.x_max*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
p2 = plot(ts, params.x_min*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
p3 = plot(ts, params.x_d*ones(size(ts,2,1)),'--k', 'LineWidth',1.5);
legend([p p2 p3], 'MPC w/ constrained states','State Constraint - x_{max/min}','Desired State - x_{des}', 'Location', 'Southeast');
xlim([min(ts) max(ts)]);
ylim([min([min(state_p),params.x_min,params.x_d])*1.3 max([max(state_p),params.x_max,params.x_d])*1.3]);
ylabel("State x_p (m)");
xlabel("Time t (s)");
set(gca,'FontSize',14);

subplot(3,1,2);
p = plot((0:sim.dt:sim.sim_t), innput,'b', 'LineWidth',1.5);
hold on;
p2 = plot(ts, params.u_max*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
p2 = plot(ts, params.u_min*ones(size(ts,2,1)),'--r', 'LineWidth',1.5);
legend([p p2], 'Control input', 'Input Constraints - u_{max/min}', 'Location', 'Northeast');
xlim([min(ts) max(ts)]);
% ylim([min([min(innput),-params.u_min])*1.3 max([max(innput),params.u_max])*1.3]);
ylim([params.u_min-0.2,params.u_max+0.2]);
ylabel("Control input u (m/s)");
xlabel("Time t (s)");

set(gca,'FontSize',14);

subplot(3,1,3);
p = stem((1:1:total_k),t_comp);
legend(p, 'MPC w/ CBF constrains', 'Location', 'Northeast');
ylabel("Computation time (s)");
xlabel("Time step k");
set(gca,'FontSize',14);  

disp(['Average computing time per time step k: ' string(mean(t_comp)) 'seconds'])


% #######################     SCRIPT END    ############################