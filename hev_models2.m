%% HEV syms model


% State derivatives (xdot)
syms vdot Pedot DPedot qdot
% States (x)
syms v Pe DPe q
% Control signals (u)
syms Ft Pestar

% Constants
syms m r0 r1 r2
syms wn xi
syms eta_gen eta_conv eta_inv eta_im eta_gear 
syms Uoc Ri Q0

% xdot = f(x,u)
vdot = Ft/m - (r2*v^2 + r1*v + r0)/m;
Pedot = DPe;
DPedot = -2*xi*wn*DPe - wn^2*Pe + wn^2*Pestar;
qdot = -(Uoc - sqrt(Uoc^2 - 4*(eta_inv*eta_im*eta_gear*Ft*v - eta_gen*eta_conv*Pe)*Ri))/(2*3600*Q0*Ri);
xdot = [vdot; Pedot; DPedot; qdot];

% Jacobians
f_x = jacobian(xdot,[v Pe DPe q]);
f_u = jacobian(xdot,[Ft Pestar]);

% Load HEV parameters
run('hev_parameters')

% Insert parameters into xdot and jacobians
xdot = subs(xdot);
f_x = subs(f_x);
f_u = subs(f_u);
%%

% Define PWA models
Ts = 1;
C = [1 0];
v0 = [10 30 50 70 90]/3.6;

for i = 1:5
    M(i).v0 = v0(i);
    M(i).DPe0 = 0;
    M(i).q0 = 0.5;
    M(i).x0 = [M(i).v0; M(i).DPe0; 0; M(i).q0];
    M(i).sol0 = solve(subs(xdot,[v DPe q],[M(i).v0 M(i).DPe0 M(i).q0]));
    M(i).Pe0 = double(M(i).sol0.Pe);
    M(i).x0(3) = M(i).Pe0;
    M(i).Ft0 = double(M(i).sol0.Ft);
    M(i).Pestar0 = double(M(i).sol0.Pestar);
    M(i).u0 = [M(i).Ft0; M(i).Pestar0];
    M(i).A = double(subs(f_x,[v DPe Pe q Ft Pestar],[M(i).v0 M(i).DPe0 M(i).Pe0 M(i).q0 M(i).Ft0 M(i).Pestar0]));
    M(i).B = double(subs(f_u,[v DPe Pe q Ft Pestar],[M(i).v0 M(i).DPe0 M(i).Pe0 M(i).q0 M(i).Ft0 M(i).Pestar0]));
    M(i).Gsys = ss(M(i).A,M(i).B,C,0);
    M(i).Gd = c2d(M(i).Gsys,Ts);
    M(i).Ad = M(i).Gd.A;
    M(i).Bd = M(i).Gd.B;
end
%%
% Define PWA models
Ts = 1;
C = [1 0];
v0 = [10 30 50 70 90]/3.6;
Pe0 = [0 200 280 330]*1000;

for i = 1:5
    for j = 1:4
        k = 4*(i-1)+j;
        M(k).v0 = v0(i);
        M(k).DPe0 = 0;
        M(k).Pe0 = Pe0(j);
        M(k).q0 = 0.5;
        M(k).x0 = [M(k).v0; M(k).DPe0; M(k).Pe0; M(k).q0];
        M(k).u0 = maple(subs(xdot,[v DPe Pe q],M(k).x0'));
        M(k).Ft0 = double(M(k).u0.Ft);
        M(k).Pestar0 = double(M(k).u0.Pestar);
        M(k).u0 = [M(k).Ft0; M(k).Pestar0];
        M(k).A = double(subs(f_x,[v DPe Pe q Ft Pestar],[M(k).v0 M(k).DPe0 M(k).Pe0 M(k).q0 M(k).Ft0 M(k).Pestar0]));
        M(k).B = double(subs(f_u,[v DPe Pe q Ft Pestar],[M(k).v0 M(k).DPe0 M(k).Pe0 M(k).q0 M(k).Ft0 M(k).Pestar0]));
        M(k).Gsys = ss(M(k).A,M(k).B,C,0);
        M(k).Gd = c2d(M(k).Gsys,Ts);
        M(k).Ad = M(k).Gd.A;
        M(k).Bd = M(k).Gd.B;
    end
end

save('PWAM','M');
