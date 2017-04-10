%% Ki-Ha E200 Specifications

% Car data
m = 39600;

% Efficiencies
eta_gen = 0.9;
eta_conv = 0.95;
eta_Pe = eta_conv*eta_gen;

eta_inv = 0.95;
eta_im = 0.9;
eta_gear = 0.98;
eta_Pout = eta_inv*eta_im*eta_gear;

% Running resistance
r0 = 554.9;
r1 = 20.3;
r2 = 3.56;

% Distance bounds (should be -inf, inf)
s_max = 100000;
s_min = -s_max;

%  Speed bounds
vb = (0:20:100)/3.6;
dv = 20/3.6;

% Tractive force upper and lower bounds
Ftb_max = [32000 32000 19000 13000 10000 7000];
Ftb_min = [-28000 -28000 -28000 -14000 -8000 -5000];

% Slope rates
Ftk_max = zeros(1,5);
Ftk_min = zeros(1,5);
for i = 1:5
    Ftk_max(i) = (Ftb_max(i+1) - Ftb_max(i))/dv;
    Ftk_min(i) = (Ftb_min(i+1) - Ftb_min(i))/dv;
end

% Engine data
% wn = 2*pi*1.4;
% wd = 2*pi*1;
% xi = sqrt(1-(wd/wn)^2);
Pe_off = 0;
Pe_low = 200000;
Pe_opt = 280000;
Pe_max = 330000;

% Battery data
Uoc = 692;
Ri = 0.1;
Q0 = 22;
q_max = 0.8;
q_min = 0.2;
Pb_max = 300000;
Pb_min = -300000;

% State and input upper and lower bounds
lb_state = [s_min; vb(1); q_min];
ub_state = [s_max; vb(6); q_max];
lb_input = [Ftb_min(1); Pe_off];
ub_input = [Ftb_max(1); Pe_max];