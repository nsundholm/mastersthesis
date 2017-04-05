function un=solvempcproblem_orig(x,F,G,M,N,Q1,Q2,ubounds,Ts,t)
warning off;
% m gives the number of control signals
[n,m]=size(G);

% Predictors for future states
[H,S]=createpredictors(F,G,N);

% Blocks repeated N times
Q1b=blockrepeat(Q1,N);
Q2b=blockrepeat(Q2,N);
Mb=blockrepeat(M,N);

% General constraints
Au=[eye(N*m);-eye(N*m)];
bu=[repmat(ubounds(:,2),N,1);repmat(-ubounds(:,1),N,1)];

% Set options and solve quadratic programming problem
options=optimset('Display','off');
U=quadprog(S'*Mb'*Q1b*Mb*S+Q2b,S'*Mb'*Q1b*Mb*H*x,Au,bu,[],[],[],[],[],options);
un=U(1:m);