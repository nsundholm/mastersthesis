%% HEV syms model

% State derivatives (xdot)
syms vdot qdot
% States (x)
syms v q 
% Control signals (u)
syms Ft Pe

% Constants
syms m r0 r1 r2
syms eta_gen eta_conv eta_inv eta_im eta_gear 
syms Uoc Ri Q0

% xdot = f(x,u)
vdot = Ft/m - (r2*v^2 + r1*v + r0)/m;
qdot = -(Uoc - sqrt(Uoc^2 - 4*(eta_inv*eta_im*eta_gear*Ft*v - eta_gen*eta_conv*Pe)*Ri))/(2*3600*Q0*Ri);
xdot = [vdot; qdot];

% Jacobians
f_x = jacobian(xdot,[v q]);
f_u = jacobian(xdot,[Ft Pe]);

% Load HEV parameters
run('hev_parameters')

% Insert parameters into xdot and jacobians
xdot = subs(xdot);
f_x = subs(f_x);
f_u = subs(f_u);

% Define PWA models
Ts = 1;
C = [0 1 0];
s0 = 0;
v0 = [10 30 50 70 90]/3.6;
q0 = 0;
As = [0 1 0; 0 0 0; 0 0 0];
Bs = [0 0; 0 0; 0 0];

for i = 1:5
    M(i).x0 = [s0; v0(i); q0];
    M(i).u0 = solve(subs(xdot,[v q],M(i).x0(2:3)'));
    M(i).Ft0 = double(M(i).u0.Ft);
    M(i).Pe0 = double(M(i).u0.Pe);
    M(i).u0 = [M(i).Ft0; M(i).Pe0];
    As(2:3,2:3) = double(subs(f_x,[v q Ft Pe],[M(i).x0(2:3)' M(i).Ft0 M(i).Pe0]));
    M(i).A = As;
    Bs(2:3,1:2) = double(subs(f_u,[v q Ft Pe],[M(i).x0(2:3)' M(i).Ft0 M(i).Pe0]));
    M(i).B = Bs;
    M(i).Gsys = ss(M(i).A,M(i).B,C,0);
    M(i).Gd = c2d(M(i).Gsys,Ts);
    M(i).Ad = M(i).Gd.A;
    M(i).Bd = M(i).Gd.B;
end

save('PWAM','M');
