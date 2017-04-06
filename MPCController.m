function uout = MPCController(currentr,currentx,oldPe,s,t)
%#ok<*AGROW>
%#ok<*REPMAT>

persistent Controller

if t == 0
    % Load PWA model struct (ugly solution, but done to avoid syms errors)
    load('PWAM.mat')
    % Load HEV parameters
    run('hev_parameters.m')
    
    % Define model data
    nx = 2; % number of states
    nu = 2; % number of inputs
    ny = 1; % number of outputs
    C = [1 0];
    
    % Define MPC controller data
    Qv = 400;
    Qq = 100000;
    Qoff = 3000;
    Qmin = 220;
    Qopt = 201;
    Qmax = 216;
    N = 20;
    
    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')
    
    % Define symbolic decision variables
    r = sdpvar(repmat(ny,1,N+1),repmat(1,1,N+1));
    x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    
    a = binvar(repmat(4,1,N),repmat(1,1,N)); % Decide Pe
    d = binvar(repmat(5,1,N),repmat(1,1,N)); % Decide M
    pastPe = binvar(1);
    stot = sdpvar(1);
    
    constraints = [];
    objective = 0;
    for k = 1:N
        % State and input upper and lower bounds
        constraints = [constraints, lb_state <= x{k} <= ub_state,...
                                    lb_input <= u{k} <= ub_input,...
                                    sum(a{k}) == 1];
                                                                        
        % Pe constraint
        u{k}(2) = a{k}(2)*Pe_low + a{k}(3)*Pe_opt + a{k}(4)*Pe_max;
        
        % System Dynamics depending on where in ss
        M1 = [x{k+1} == M(1).x0 + M(1).Ad*(x{k} - M(1).x0) + M(1).Bd*(u{k} - M(1).u0), vb(1) <= x{k}(1) <= vb(2), Ftb_min(1) + Ftk_min(1)*(x{k}(1) - vb(1)) <= u{k}(1) <= Ftb_max(1) + Ftk_max(1)*(x{k}(1) - vb(1)), Pb_min <= eta_Pout*(10/3.6)*u{k}(1) - eta_Pe*u{k}(2) <= Pb_max];
        M2 = [x{k+1} == M(2).x0 + M(2).Ad*(x{k} - M(2).x0) + M(2).Bd*(u{k} - M(2).u0), vb(2) <= x{k}(1) <= vb(3), Ftb_min(2) + Ftk_min(2)*(x{k}(1) - vb(2)) <= u{k}(1) <= Ftb_max(2) + Ftk_max(2)*(x{k}(1) - vb(2)), Pb_min <= eta_Pout*(30/3.6)*u{k}(1) - eta_Pe*u{k}(2) <= Pb_max];
        M3 = [x{k+1} == M(3).x0 + M(3).Ad*(x{k} - M(3).x0) + M(3).Bd*(u{k} - M(3).u0), vb(3) <= x{k}(1) <= vb(4), Ftb_min(3) + Ftk_min(3)*(x{k}(1) - vb(3)) <= u{k}(1) <= Ftb_max(3) + Ftk_max(3)*(x{k}(1) - vb(3)), Pb_min <= eta_Pout*(50/3.6)*u{k}(1) - eta_Pe*u{k}(2) <= Pb_max];
        M4 = [x{k+1} == M(4).x0 + M(4).Ad*(x{k} - M(4).x0) + M(4).Bd*(u{k} - M(4).u0), vb(4) <= x{k}(1) <= vb(5), Ftb_min(4) + Ftk_min(4)*(x{k}(1) - vb(4)) <= u{k}(1) <= Ftb_max(4) + Ftk_max(4)*(x{k}(1) - vb(4)), Pb_min <= eta_Pout*(70/3.6)*u{k}(1) - eta_Pe*u{k}(2) <= Pb_max];
        M5 = [x{k+1} == M(5).x0 + M(5).Ad*(x{k} - M(5).x0) + M(5).Bd*(u{k} - M(5).u0), vb(5) <= x{k}(1) <= vb(6), Ftb_min(5) + Ftk_min(5)*(x{k}(1) - vb(5)) <= u{k}(1) <= Ftb_max(5) + Ftk_max(5)*(x{k}(1) - vb(5)), Pb_min <= eta_Pout*(90/3.6)*u{k}(1) - eta_Pe*u{k}(2) <= Pb_max];
        
        constraints = [constraints, implies(d{k}(1), M1),...
                                    implies(d{k}(2), M2),...
                                    implies(d{k}(3), M3),...
                                    implies(d{k}(4), M4),...
                                    implies(d{k}(5), M5),...
                                    sum(d{k}) == 1];
                                
        % Objective function                       
        objective = objective + (C*x{k} - r{k})'*Qv*(C*x{k} - r{k}) + (x{k}(2) - 0.5)'*Qq*stot*(x{k}(2) -0.5) + Qoff*abs(a{k}(1) - pastPe) + Qmin*a{k}(2) + Qopt*a{k}(3) + Qmax*a{k}(4);
        pastPe = a{k}(1);
    end
    
    constraints = [constraints, lb_state <= x{k+1} <= ub_state];
    
    % Define an optimizer that solves the problem for a particular initial
    % state and reference.
    ops = sdpsettings('verbose',1,'solver','cplex');
    parameters_in = {x{1},[r{:}],pastPe,stot};
    solutions_out = {u{1},a{1}(1)};
    Controller = optimizer(constraints, objective, ops, parameters_in, solutions_out)
    
    [solutions,diagnostics] = Controller(currentx,currentr,oldPe,s);
    if diagnostics == 1
        error('The problem is infeasible');
    end
    uout = [solutions{1}; solutions{2}];
else
    s = (s/14850)^2;
    [solutions,diagnostics] = Controller(currentx,currentr,oldPe,s);
    if diagnostics == 1
        error('The problem is infeasible');
    end
    uout = [solutions{1}; solutions{2}];
end
