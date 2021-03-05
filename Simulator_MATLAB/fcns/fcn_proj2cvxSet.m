function x = fcn_proj2cvxSet(u,A,b)
    H = eye(2);
    f = -u(:);
    Aeq = [];
    beq = [];
    
    %options = optimoptions('quadprog','Display','off');
    options = [];
    x = quadprog(H,f,A,b,Aeq,beq,[],[],[],options);
end