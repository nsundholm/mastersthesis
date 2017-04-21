%% HEV syms model

% State derivatives (xdot)
syms vdot Pedot qdot
% States (x)
syms v Pe q 
% Control signals (u)
syms Ft Fmb Pes

% Constants
syms m r0 r1 r2
syms eta_gen eta_conv eta_inv eta_im eta_gear
syms Uoc Ri Q0 Paux TPe

% xdot = f(x,u)
vdot = Ft/m - Fmb/m - (r2*v^2 + r1*v + r0)/m;
Pedot = (Pes - Pe)/TPe; 
qdot = -(Uoc - sqrt(Uoc^2 - 4*(eta_inv*eta_im*eta_gear*Ft*v - eta_gen*eta_conv*Pe + Paux)*Ri))/(2*3600*Q0*Ri);
xdot = [vdot; Pedot; qdot];

% Jacobians
f_x = jacobian(xdot,[v Pe q]);
f_u = jacobian(xdot,[Ft Fmb Pes]);

% Load HEV parameters
run('hev_parameters')

% Insert parameters into xdot and jacobians
xdot = subs(xdot);
f_x = subs(f_x);
f_u = subs(f_u);

% Define PWA models
Ts = 1;
C = [0 0 0 0];

s0 = 0;
v0 = [10 30 50 70 90]/3.6;
Pe0 = 0;
q0 = 0;
Fmb0 = 0;

A = [0 1 0 0;
     0 0 0 0;
     0 0 -(1/TPe) 0;
     0 0 0 0];
B = [0 0 0;
     0 0 0;
     0 0 (1/TPe);
     0 0 0];

for i = 1:5
    sol = solve(subs(xdot,[v q Fmb],[v0(i) q0 Fmb0]));
    M(i).x0 = [s0; v0(i); double(sol.Pe); q0];
    M(i).u0 = [double(sol.Ft); Fmb0; double(sol.Pes)];

    fx = double(subs(f_x,[v Pe q Ft Fmb Pes],[M(i).x0(2:4)' M(i).u0']));
    A(2,2:4) = fx(1,:);
    A(4,2:4) = fx(3,:);
    M(i).A = A;
    fu = double(subs(f_u,[v Pe q Ft Fmb Pes],[M(i).x0(2:4)' M(i).u0']));
    B(2,:) = fu(1,:);
    B(4,:) = fu(3,:);
    M(i).B = B;
    
    M(i).Gsys = ss(A,B,C,0);
    M(i).Gd = c2d(M(i).Gsys,Ts);
    M(i).Ad = M(i).Gd.A;
    M(i).Bd = M(i).Gd.B;
end

save('PWAM','M');
