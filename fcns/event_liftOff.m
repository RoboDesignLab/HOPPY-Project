function [zeroCrossing,isterminal,direction] = event_liftOff(t,X,p)

Tst = p.Tst;
tTD = p.tTD(1,end);
ptTD = p.ptTD;          % touchdown toe position
params = p.params;
M_r = p.M_r;
B_EMF = p.B_EMF;

if ~p.isMotorDynamics   % if not adding motor dynamics
    M_r = 0*M_r;
    B_EMF = 0*B_EMF;
end

q = X(1:4);
dq = X(5:8);
De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

% Holonomic constraints
Jhc = fcn_Jhc(q,ptTD,params);
dJhc = fcn_dJhc(q,dq,ptTD,params);

s = (t - tTD) / Tst;
s(s<0) = 0;
s(s>1) = 1;

% Force profile using Bezier polynomials
Fx = polyval_bz(p.Fx_co_bz, s);
Fz = polyval_bz(p.Fz_co_bz, s);

tau_fb = p.Kp_st * (p.q_d - q(3:4)) + p.Kd_st * (-dq(3:4));

%Joint-level control
Jc_HIP = fcn_J_toe_HIP(q,params);
%Torques of joints 3 (hip) and 4 (knee)
u_ = -Jc_HIP'*[Fx; Fz] + tau_fb;    

% Solve the linear system:
% De * ddq + Ce * dq + Ge = Jhc' * GRF + Be * u (4 eqns)
% Jhc * ddq + dJhc * dq = 0 (2 eqns)
% [De  -Jhc'] * [ddq] = [Be*u - Ce*dq - Ge]
% [Jhc  0   ]   [GRF]   [-dJhc*dq         ]
% unknowns: ddq(4x1), GRF(2x1) 
% control: u(2x1)
tau_s = -0.0242 * q(4) + 0.0108;    %Spring torque
v_taus = [0;0;0;2*tau_s];

Amat = [De + M_r, -Jhc'; 
        Jhc, zeros(2,2)];

bvec = [Be * u_ - (Ce+B_EMF)*dq - Ge + v_taus; 
        -dJhc * dq];

ddqu = Amat \ bvec;
ddq = ddqu(1:4);
GRF = reshape(ddqu(5:6),[1,2]);

GRFz = GRF(2);

% the liftoff event happens when the z-component of GRF is less than a
% threshold (a small number relative to the GRF)
zeroCrossing =  GRFz - 1.5;
isterminal   =  1;
direction    =  -1;

end


