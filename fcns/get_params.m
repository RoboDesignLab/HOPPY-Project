function p = get_params()

p.Tst = 0.35;                % nominal stance time
p.Rboom = 556.0e-3;         % boom radius
g = 9.81;                   % Gravity
p.g = g;

% The Bezier coefficient for the GRF profile, the first and last entries 
% should be 0 for physical feasibility
p.Fx_co_bz = [0 0 -25 0 0];
p.Fz_co_bz = [0 20 100 0 0];

% % The following piece of code plots the shape of GRF
% s = linspace(0,1,101);
% Fx = polyval_bz(p.Fx_co_bz,s);
% Fz = polyval_bz(p.Fz_co_bz,s);
% hold on
% plot(s*p.Tst,Fx,'b')
% plot(s*p.Tst,Fz,'r')
% legend('Fx','Fz')
% xlabel('Times [s]')
% ylabel('GRF [N]')


% stance phase
% a soft joint PD control is applied to stabilize the stance leg
p.Kp_st = 0.03;             % proportional gain
p.Kd_st = 0.08;             % derivative gain
p.q_d = [pi/3; -pi/2];      % desired joint position

% swing phase parameters
p.Kp_sw = diag(150*[1 1]);  % proportional gain
p.Kd_sw = diag(5*[1 1]);    % derivative gain
p.Krh = 0.1;                % constant for raibert hopper

% animation parameter
p.N_animate = 10;           % for animation slow-motion.

% actuator dynamics
Nh = 26.9;                  % gear ratio at the hip motor
Nk = 28.8;                  % gear ratio at the knee motor
I_rotor = 7e-6;             % motor rotor inertia
Rw = 1.3;                   % coil resistance
kT = 0.0135;                % motor constant
kv = 0.0186;                % speed constant

% damping from back-emf
Bh = kv * kT * Nh^2 / Rw;
Bk = kv * kT * Nk^2 / Rw;
M_r = I_rotor * [zeros(2) zeros(2);
                 zeros(2) diag([Nh^2,Nk^2])];
B_EMF = [zeros(2) zeros(2);
          zeros(2) diag([Bh,Bk])];

p.M_r = M_r;
p.B_EMF = B_EMF;

% control limit
p.uHip.A = [0 1;0 -1;0.1391 1;-0.1391 -1];
p.uHip.b = [4.3578;4.3578;3.3356;3.3356];
p.uHip.V = [-55.3084    4.3578;
           -7.3487    4.3578;
           55.3084   -4.3578;
            7.3487   -4.3578];
        
p.uKnee.A = [0 1;0 -1;0.1594 1;-0.1594 -1];
p.uKnee.b = [4.6656;4.6656;3.5712;3.5712];
p.uKnee.V = [-51.6738    4.6656;
               -6.8657    4.6656;
               51.6738   -4.6656;
                6.8657   -4.6656];

%% parameters for matrix calculation
HB = 196.5e-3;%245e-3;            %Link lengths
LB = 556.0e-3;%440e-3;
DB = 48e-3;
LH = 96e-3;
LK = 154.5e-3;
DK = 52e-3;

M1 = 0.268;%0.172;     %Link masses
M2 = 2.365;%0.738;
M3 = 0.656;%0.304;
M4 = 0.149;%0.123;

%Position of the CoM 1 in link frame 1 (constant)
rx1 = -0.00056660;      
ry1 = 0.0;
rz1 = -0.06176511;

%Position of the CoM 2 in link frame 2 (constant)
rx2 = -0.50195830;%-0.143; 
ry2 = -0.03678363;%-0.032;
rz2 = 0.00001342;%0.0;
    
rx3 = 0.04825821;%0.032;
ry3 = 0.00027269;%0.0;
rz3 = 0.07708701;%0.068;
    
rx4 = 0.00207136;%0.001;
ry4 = 0.02086667;%0.020;
rz4 = 0.14511501;%0.139;

% Inertia tensor of link one taken at the center of mass and aligned with 
% the B coordinate system.
Jxx1 = 0.00115952;%0.00064578; 
Jyy1 = 0.00104649;%0.00062034; 
Jzz1 = 0.00030518;%0.00017348; 
Jxy1 = 0;
Jxz1 = -0.00000703;%0.00000593;
Jyz1 = 0;

Jxx2 = 0.00270252;%0.00076962;
Jyy2 = 0.30208952;%0.02261852;
Jzz2 = 0.30305924;%0.02306310;
Jxy2 = 0.01161483;%0.00116033;
Jxz2 = -0.00006434;%-0.0000165;
Jyz2 = -0.00000838;%0.00000022;

Jxx3 = 0.00082110;%0.00045696;
Jyy3 = 0.00235762;%0.00108532;
Jzz3 = 0.00168340;%0.00070883;
Jxy3 = -0.00002821;%-0.00000525;
Jxz3 = 0.00058518;%0.00026769;
Jyz3 = -0.00001795;%-0.00000651;

Jxx4 = 0.00039424;%0.00039564;
Jyy4 = 0.00032191;%0.00033576;
Jzz4 = 0.00010442;%0.00008091;
Jxy4 = -0.00000379;%-0.00000437;
Jxz4 = -0.00000066;%0.00000366;
Jyz4 = 0.00003269;%0.00003475;

% % parameters for matrix calculations
% [g, HB, LB, DB, LH, DK, LK, M1, M2, M3, M4, ...
%        rx1, ry1, rz1, rx2, ry2, rz2, rx3, ry3, rz3, rx4, ry4, rz4, J1, J2, J3, J4] = fcn_params;

p.params = [g, HB, LB, DB, LH, DK, LK, M1, M2, M3, M4, ...
    rx1, ry1, rz1, rx2, ry2, rz2, rx3, ry3, rz3, rx4, ry4, rz4, ...
    Jxx1, Jyy1, Jzz1, Jxy1, Jxz1, Jyz1,...
    Jxx2, Jyy2, Jzz2, Jxy2, Jxz2, Jyz2,...
    Jxx3, Jyy3, Jzz3, Jxy3, Jxz3, Jyz3,...
    Jxx4, Jyy4, Jzz4, Jxy4, Jxz4, Jyz4];
    
    
end
    
    