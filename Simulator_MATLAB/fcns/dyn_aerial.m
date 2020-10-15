function [dXdt,u,F] = dyn_aerial(t,X,p)

params = p.params;
Kp = p.Kp_sw;              % proportional gain
Kd = p.Kd_sw;              % derivative gain
Krh = p.Krh;               % constant for raibert hopper
M_r = p.M_r;
B_EMF = p.B_EMF;
if ~p.isMotorDynamics   % if not adding motor dynamics
    M_r = 0*M_r;
    B_EMF = 0*B_EMF;
end

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
    q = X(1:4,ii);                          % joint positions
    dq = X(5:8,ii);                         % joint velocities

    Me = fcn_De(q,params);                  % Inertia matrix
    Ce = fcn_Ce(q,dq,params);               % Coriolis matrix
    Ge = fcn_Ge(q,params);                  % Gravity vector
    Be = fcn_Be(q,params);                  % Actuation selection matrix

    % swing controller
    vx = dq(1) * p.Rboom;                   % forward velocity
    p_toe_HIP_d = [Krh * vx;-0.15];         % desired swing foot position
    p_toe_HIP = fcn_p_toe_HIP(q,params);    % real swing foot position
    
    J_toe_HIP = fcn_J_toe_HIP(q,params);
    v_toe_HIP = J_toe_HIP * dq(3:4);        % real swing foot velocity
    
    % PD control on swing foot
    F_sw = Kp * (p_toe_HIP_d - p_toe_HIP) + Kd * (-v_toe_HIP);
    u_ = J_toe_HIP' * F_sw;
    
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
    
    u(ii,:) = u_';              % applied joint torque
    F(ii,:) = F_sw';

    % spring effect
    tau_s = -0.0242 * q(4) + 0.0108;
    v_taus = [0;0;0;2*tau_s];

    % dynamics
    % (Me + Mr)*ddq + (C+B_EMF)*dq + G = Be*u + v_taus
    ddq = (Me+M_r) \ (Be * u_ + v_taus - (Ce + B_EMF)*dq - Ge);

    dXdt(:,ii) = [dq;ddq];
end

end
