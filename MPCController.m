function uout = MPCController(currentr,currentx,t)
%#ok<*AGROW>
%#ok<*REPMAT>

persistent Controller
persistent uold
persistent aold

if t == 0
    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')
    
    % Load PWA model struct (ugly solution, but done to avoid syms errors)
    load('PWAM.mat')
    % Load HEV parameters
    run('hev_parameters.m')
    
    N = 20;
    Mb = 20;
    mb = ones(1,N);
    for i=1:length(mb)
        if (mod(i-1,Mb) == 0)
            mb(1,i) = 0;
        end
    end
    
    % Define model data
    nx = 4; % number of states
    nu = 3; % number of inputs
    ny = 1; % number of outputs
    
    % Define MPC controller data
    Qv = 1000;
    Qq = intvar(1);
    QFmb = 0.001;
    Qoff = 20;
    Qmin = 2.20;
    Qopt = 2.01;
    Qmax = 2.16;
    
    % Define symbolic decision variables
    r = sdpvar(repmat(ny,1,N+1),repmat(1,1,N+1));
    x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    
    a = binvar(repmat(4,1,N),repmat(1,1,N)); % Decide Pes
    d = binvar(repmat(5,1,N),repmat(1,1,N)); % Decide M
    pastPes = binvar(1);
    
    constraints = [];
    objective = 0;
    for k = 1:N
        % State and input upper and lower bounds
        constraints = [constraints, lb_state <= x{k} <= ub_state,...
                                    lb_input <= u{k} <= ub_input];
                                
%         % Move-blocking
%         if mb(k) == 1
%             constraints = [constraints, a{k} == a{k-1}];
%         end
                                                                        
        % Pe constraint
        u{k}(3) = a{k}(1)*Pes_off + a{k}(2)*Pes_low + a{k}(3)*Pes_opt + a{k}(4)*Pes_max; 
        constraints = [constraints, sum(a{k}) == 1];
        
        % System Dynamics depending on where in ss
        M1 = [x{k+1} == M(1).x0 + M(1).Ad*(x{k} - M(1).x0) + M(1).Bd*(u{k} - M(1).u0), vl(1) <= x{k}(2) <= vl(2), Ftl_min(1) + Ftlk_min(1)*(x{k}(2) - vl(1)) <= u{k}(1) <= Ftl_max(1) + Ftlk_max(1)*(x{k}(2) - vl(1)), u{k}(2) == 0,                                            Pb_min <= eta_Pout*vl(2)*u{k}(1) - eta_Pe*u{k}(3) <= Pb_max];
        M2 = [x{k+1} == M(2).x0 + M(2).Ad*(x{k} - M(2).x0) + M(2).Bd*(u{k} - M(2).u0), vl(2) <= x{k}(2) <= vl(3), Ftl_min(2) + Ftlk_min(2)*(x{k}(2) - vl(2)) <= u{k}(1) <= Ftl_max(2) + Ftlk_max(2)*(x{k}(2) - vl(2)), u{k}(2) == 0,                                            Pb_min <= eta_Pout*vl(3)*u{k}(1) - eta_Pe*u{k}(3) <= Pb_max];
        M3 = [x{k+1} == M(3).x0 + M(3).Ad*(x{k} - M(3).x0) + M(3).Bd*(u{k} - M(3).u0), vl(3) <= x{k}(2) <= vl(4), Ftl_min(3) + Ftlk_min(3)*(x{k}(2) - vl(3)) <= u{k}(1) <= Ftl_max(3) + Ftlk_max(3)*(x{k}(2) - vl(3)), u{k}(2) <= Fmbl_max(3) + Fmblk_max(3)*(x{k}(2) - vl(3)), Pb_min <= eta_Pout*vl(4)*u{k}(1) - eta_Pe*u{k}(3) <= Pb_max];
        M4 = [x{k+1} == M(4).x0 + M(4).Ad*(x{k} - M(4).x0) + M(4).Bd*(u{k} - M(4).u0), vl(4) <= x{k}(2) <= vl(5), Ftl_min(4) + Ftlk_min(4)*(x{k}(2) - vl(4)) <= u{k}(1) <= Ftl_max(4) + Ftlk_max(4)*(x{k}(2) - vl(4)), u{k}(2) <= Fmbl_max(4) + Fmblk_max(4)*(x{k}(2) - vl(4)), Pb_min <= eta_Pout*vl(5)*u{k}(1) - eta_Pe*u{k}(3) <= Pb_max];
        M5 = [x{k+1} == M(5).x0 + M(5).Ad*(x{k} - M(5).x0) + M(5).Bd*(u{k} - M(5).u0), vl(5) <= x{k}(2) <= vl(6), Ftl_min(5) + Ftlk_min(5)*(x{k}(2) - vl(5)) <= u{k}(1) <= Ftl_max(5) + Ftlk_max(5)*(x{k}(2) - vl(5)), u{k}(2) <= Fmbl_max(5) + Fmblk_max(5)*(x{k}(2) - vl(5)), Pb_min <= eta_Pout*vl(6)*u{k}(1) - eta_Pe*u{k}(3) <= Pb_max];
        
        constraints = [constraints, implies(d{k}(1), M1),...
                                    implies(d{k}(2), M2),...
                                    implies(d{k}(3), M3),...
                                    implies(d{k}(4), M4),...
                                    implies(d{k}(5), M5),...
                                    sum(d{k}) == 1];
                                
        % Objective function                       
        objective = objective + (x{k}(2)-r{k})'*Qv*(x{k}(2)-r{k}) +...
                                Qq*-x{k}(4) +...
                                QFmb*u{k}(2) +...
                                Qoff*abs(a{k}(1)-pastPes) + Qmin*a{k}(2) + Qopt*a{k}(3) + Qmax*a{k}(4);
        pastPes = a{k}(1);
    end
    
    constraints = [constraints, lb_state <= x{N+1} <= ub_state];
    
    % Define an optimizer that solves the problem for a particular initial
    % state and reference.
    ops = sdpsettings('verbose',0,'solver','gurobi','gurobi.MIPGap',0.0001);
    parameters_in = {x{1},[r{:}],pastPes,Qq};
    solutions_out = {u{1},a{1}(1)};
    Controller = optimizer(constraints, objective, ops, parameters_in, solutions_out)

    [solutions,diagnostics] = Controller(currentx,currentr,0,0);
    if diagnostics == 1
        error('The problem is infeasible');
    end
    uout = solutions{1};
    aold = solutions{2};
    uold = uout;
else        
    if mod(t,1) == 0
        if currentx(1) >= 3500
            [solutions,diagnostics] = Controller(currentx,currentr,round(aold),10000);
        else
            [solutions,diagnostics] = Controller(currentx,currentr,round(aold),0);
        end
        if diagnostics == 1
            error('The problem is infeasible');
        end
        uout = solutions{1};
        aold = solutions{2};
        uold = uout;
    else
        uout = uold;
    end
end
