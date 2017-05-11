%% Ki-Ha E200 Specifications
N = 40;
% Efficiencies
eta_gen = 0.9;
eta_conv = 0.95;
eta_Pe = eta_conv*eta_gen;
eta_inv = 0.95;
eta_im = 0.9;
eta_gear = 0.98;
eta_Pout = eta_inv*eta_im*eta_gear;

% Car data
mv = 39600;
mp = 117*60;
iv = 1.1;
m = (mv+mp)*iv;
maxacc = 2.3/3.6;
maxdec = -2.0/3.6; 
Fstart = m*3*9.8/1000; % 3 kgf/ton
r0 = 554.9;
r1 = 20.3;
r2 = 3.56;
Paux = 30000;

% Engine data
Pes_off = 0;
Pes_low = 200000;
Pes_opt = 280000;
Pes_max = 330000;
TPe = 0.2;

% Battery data
Uoc = 692;
Ri = 0.1;
Q0 = 22;
Pb_max = 300000;
Pb_min = -300000;

% State bounds
s_max = 20000;
s_min = -100;
v_max = 100/3.6;
v_min = 0;
Pe_max = 330000;
Pe_min = 0;
q_max = 0.8;
q_min = 0.2;

% Non-Linear Ft & Fmb upper and lower bounds
vtmb = 0:100;
Ft_max = zeros(1,101);
Ft_min = zeros(1,101);

for i=1:101
    % Ftmax
    if vtmb(i) <= 20
        Ft_max(i) = m*maxacc + Fstart;
    elseif vtmb(i) <= 70
        Ft_max(i) = Ft_max(21)*(20/vtmb(i));
    else
        Ft_max(i) = Ft_max(21)*(20/vtmb(i))*(70/vtmb(i));
    end
    %Ftmin
    if vtmb(i) <= 42
        Ft_min(i) = m*maxdec;
    else
        Ft_min(i) = m*maxdec*(42/vtmb(i));
    end
end

Fmb_max = Ft_min - Ft_min(1);
Fmb_min = 0;

% figure;
% plot(vtmb,Ft_max,vtmb,Ft_min,vtmb,Fmb_max);

% Linear Ft & Fmb upper and lower bounds
vl = vtmb(1:20:101)/3.6;
Ftl_max = Ft_max(1:20:101);
Ftl_min = Ft_min(1:20:101);
Fmbl_max = Ftl_min - Ftl_min(1);
Fmbl_min = 0;

for i = 1:5
    Ftlk_max(i) = (Ftl_max(i+1) - Ftl_max(i))/(20/3.6);
    Ftlk_min(i) = (Ftl_min(i+1) - Ftl_min(i))/(20/3.6);
    Fmblk_max(i) = (Fmbl_max(i+1) - Fmbl_max(i))/(20/3.6);
end

% State and input upper and lower bounds
lb_state = [s_min; v_min; Pe_min; q_min];
ub_state = [s_max; v_max; Pe_max; q_max];
lb_input = [Ftl_min(1); Fmbl_min; Pes_off];
ub_input = [Ftl_max(1); Fmbl_max(end); Pes_max];