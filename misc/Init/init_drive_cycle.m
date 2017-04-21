%% init_drive_cycle.m

global h;
h = Ts;

% Drive cycle generation
x = [0, 50, 100];       %[s]
y = [0, 100/3.6, 0];    %[km/h]
T_z = linspace(min(x), max(x), (max(x)-min(x))/h + 1)';
V_z = spline(x,y,T_z);