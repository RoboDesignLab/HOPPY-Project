% Hopping leg with boom
% Author: Yanran Ding and João Ramos 
% Last modified: 2020/10
clc;clear all;close all

addpath gen         % fcns to calculate matrices for kinematics & dynamics
addpath fcns        % other utility functions

disp('------ Accompanying Code for the paper: -------')
disp('HOPPY: An open-source and low-cost kit for dynamic robotics education')
disp('Joao Ramos, Yanran Ding, Youngwoo Kim, Kevin Murphy, and Daniel Block')
disp('-------------------------------------------------')
disp('Initializing ..............')

% --- parameters ---
p = get_params();           % Getting physical parameters of the robot              
Nstep = 15;                 % number of Hops
p.Nstep = Nstep;
p.isMotorDynamics = 1;      % 1-add motor dynamics, 0-no motor dynamics
p.isControlSaturate = 1;    % Add control satuaturation, 0-No saturation
if p.isMotorDynamics == 1
    disp('Simulating WITH the motor dynamics')
else
    disp('Simulating WITHOUT the motor dynamics')
end

if p.isControlSaturate == 1
    disp('Simulating WITH the control constraints')
else
    disp('Simulating WITHOUT the control constraints')
end

% Initial condition
q0 = [0; 0; pi/3; -pi/2];   % initial Joint angles
dq0 = [0; 0; 0; 0];         % initial Joint velocities
ic = [q0; dq0];

% simulation time & Data Recording
tstart = 0;
tfinal = 2 * Nstep;         % Maximum simulation time
tout = tstart;
Xout = ic';
Uout = [0,0];
Fout = [0,0];

p.tTD = zeros(2,0);

for istep = 1:Nstep
    %% aerial phase
    options = odeset('Events',@(t,X)event_touchDown(t,X,p));
    [t,X] = ode45(@(t,X)dyn_aerial(t,X,p),[tstart, tfinal], Xout(end,:),options);

    p.tTD(:,end+1) = [t(end);0];                             % touchdown time
    p.ptTD = fcn_p_toe(X(end,1:4),p.params);    % touchdown toe pos

    % data logging
    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    [~,u,F] = dyn_aerial(t,X,p);
    Uout = [Uout;u(2:nt,:)];
    Fout = [Fout;F(2:nt,:)];
    tstart = tout(end);
    
    %% Impact map (hard contact)
    X_prev = Xout(end,:);
    X_post = fcn_impactMap(X_prev,p);
    Xout(end,:) = X_post';

    %% stance phase
    options = odeset('Events',@(t,X)event_liftOff(t,X,p));
    [t,X] = ode45(@(t,X)dyn_stance(t,X,p),[tstart, tfinal], Xout(end,:), options);
    p.tLO = t(end);
    p.tTD(2,end) = t(end);

    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    [~,u,F] = dyn_stance(t,X,p);
    Uout = [Uout;u(2:nt,:)];
    Fout = [Fout;F(2:nt,:)];
    tstart = tout(end);

    fprintf('%d out of %d steps complete!\n',istep,Nstep)
end
fprintf('Simulation Complete!\n')

%% Visualing the motion
% If you want to generate a .mp4 movie file
p.boolAnimate = 1;
[t,COM,HIP,FOOT] = animateRobot(tout,Xout,Uout,Fout,p);

