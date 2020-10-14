function [dXdt,u,F] = dyn_stance(t,X,p)

% --- parameters ---
params = p.params;
tTD = p.tTD(1,end);            % touchdown time
ptTD = p.ptTD;          % touchdown toe position
Tst = p.Tst;
M_r = p.M_r;
B_EMF = p.B_EMF;

if ~p.isMotorDynamics   % if not adding motor dynamics
    M_r = 0*M_r;
    B_EMF = 0*B_EMF;
end
      
% --- get matrices ---
[m,n] = size(X);
if n == 8
    X = X';
end

N = size(X,2);
u = zeros(N,2);
F = zeros(N,2);
dXdt = zeros(size(X,1),N);

% For the integrator ode45, N is 1, for evaluating tau and the GRF, it will 
% be as long as the input vector
for ii = 1:N 
    
    q = X(1:4,ii);
    dq = X(5:8,ii);
    De = fcn_De(q,params);
    Ce = fcn_Ce(q,dq,params);
    Ge = fcn_Ge(q,params);
    Be = fcn_Be(q,params);

    % Holonomic constraints
    Jhc = fcn_Jhc(q,ptTD,params);
    dJhc = fcn_dJhc(q,dq,ptTD,params);

    %% Controller
    % Feedforward force:
    s = (t(ii) - tTD) / Tst;        % stance phase parametrization s = [0, 1]
    s(s<0) = 0;s(s>1) = 1;
    % Force profile using Bezier polynomials
    Fx = polyval_bz(p.Fx_co_bz, s);
    Fz = polyval_bz(p.Fz_co_bz, s);
    
    % Feedback torque: 
    % a soft joint PD control is applied to stabilize the stance leg
    tau_fb = p.Kp_st * (p.q_d - q(3:4)) + p.Kd_st * (-dq(3:4));

    %Joint-level control
    Jc_HIP = fcn_J_toe_HIP(q,params);
    %Torques of joints 3 (hip) and 4 (knee)
    u_ = -Jc_HIP'*[Fx; Fz] + tau_fb;
    
    
    
    % --------- torque saturation --------------
    if p.isControlSaturate
%         [A_w,A_u] * [w;tau] <= b
%         [AuH, 0][u_H] <= [bH - AwH * wH]
%         [0, AuK][u_K]    [bK - AwK * wK]
        [AwH,AuH,bH,AwK,AuK,bK] = deal(p.uHip.A(:,1),p.uHip.A(:,2),p.uHip.b,...
                                    p.uKnee.A(:,1),p.uKnee.A(:,2),p.uKnee.b);
        wH = X(7);
        wK = X(8);
        Am = blkdiag(AuH,AuK);
        bm = [bH - AwH * wH;
              bK - AwK * wK];
        if any(Am * u_ >= bm)
            u_ = fcn_proj2cvxSet(u_,Am,bm);
        end
    end
    % ------------------------------------------
    u(ii,:) = u_';
    
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
    F(ii,:) = reshape(ddqu(5:6),[1,2]);

    dXdt(:,ii) = [dq; ddq];
    
end


end


