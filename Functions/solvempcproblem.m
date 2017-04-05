function un=solvempcproblem(x,F,G,M,N,Q1,Q2,ubounds,Ts,P_req,zbounds,t)
warning off;
% m gives the number of control signals
[n,m]=size(G);

% Predictors for future states
[H,S]=createpredictors(F,G,N);

% Q1, Q2, M matrices repeated N times
Q1b=blockrepeat(Q1,N);
%Q2b=blockrepeat(Q2,N);
Mb=blockrepeat(M,N);

Q2b = Q2*ones(N,1);

% Constraints A*U <= b
Au=[eye(N*m);-eye(N*m)];
bu=[repmat(ubounds(:,2),N,1);repmat(-ubounds(:,1),N,1)];
Az=[Mb*S;-Mb*S];
bz=[repmat(zbounds(:,2),N,1)-Mb*H*x;repmat(-zbounds(:,1),N,1)+M*H*x];

% Constraint Aeq*x =beq
%Aeq = S(end,:);
%beq = -H(end,:)*x;

% Set options and solve quadratic programming problem
options=optimset('Display','off');
U = quadprog(S'*Mb'*Q1b*Mb*S,S'*Mb'*Q1b*Mb*H*x,[Au;Az],[bu;bz],[],[],[],[],[],options);
un=U(1:m);
