%% HEV syms model

% State derivatives (xdot)
syms sdot vdot Pedot qdot
% States (x)
syms s v Pe q 
% Control signals (u)
syms Ft Fmb Pes

% Constants
syms m r0 r1 r2
syms eta_gen eta_conv eta_inv eta_im eta_gear
syms Uoc Ri Q0 Paux TPe

% xdot = f(x,u)
sdot = v;
vdot = Ft/m - Fmb/m - (r2*v^2 + r1*v + r0)/m;
Pedot = (Pes - Pe)/TPe; 
qdot = -(Uoc - sqrt(Uoc^2 - 4*(Ft*v - eta_gen*eta_conv*Pe + Paux)*Ri))/(2*3600*Q0*Ri);
xdot = [sdot; vdot; Pedot; qdot];

% Jacobians
f_x = jacobian(xdot,[s v Pe q]);
f_u = jacobian(xdot,[Ft Fmb Pes]);

% Load HEV parameters
run('hev_parameters')

% Insert parameters into xdot and jacobians
xdot = subs(xdot);
f_x = subs(f_x);
f_u = subs(f_u);

% Define PWA models
s0 = 0;
v0 = [10 30 50 70 90]/3.6;
Pe0 = 0;
q0 = 0;

for i = 1:5
    sol = solve(subs(xdot(2:4,:),[s v Pe q],[s0 v0(i) Pe0 q0]));
    M(i).x0 = [s0; v0(i); Pe0; q0];
    M(i).u0 = [double(sol.Ft); double(sol.Fmb); double(sol.Pes)];

    M(i).A = double(subs(f_x,[s v Pe q Ft Fmb Pes],[M(i).x0' M(i).u0']));
    M(i).B = double(subs(f_u,[s v Pe q Ft Fmb Pes],[M(i).x0' M(i).u0']));
    M(i).C = [0 0 0 0];
    
    M(i).Gsys = ss(M(i).A,M(i).B,M(i).C,0);
    M(i).Gd = c2d(M(i).Gsys,1);
    M(i).Ad = M(i).Gd.A;
    M(i).Bd = M(i).Gd.B;
end

save('PWAM','M');
