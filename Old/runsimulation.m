%%
clear;
close all;
sim('hevmpc.slx');









%% Drive Cycle 
fig = figure; %#ok<*NASGU>
plot(t,x_tot);
title('Total Distance x_{tot}');
xlabel('Time [s]');
ylabel('x_{tot} [m]');

fig = figure;
plot(t,v*3.6);
title('Vehicle Speed v');
xlabel('Time [s]');
ylabel('v [km/h]')

fig = figure;
plot(t,dv);
title('Vehicle Acceleration a');
xlabel('Time [s]');
ylabel('a [{m}/{s^2}]')

%% Wheel
fig = figure;
plot(w_wheel);
title('');
xlabel('Time [s]');
ylabel('w_wheel [rad/s]')

fig = figure;
plot(P_wheel);
title('');
xlabel('Time [s]');
ylabel('P_wheel [W]')

fig = figure;
plot(T_wheel);
title('');
xlabel('Time [s]');
ylabel('T_wheel [Nm]')

%% Wheel - EG Transmission
fig = figure;
plot(w_trans_1);
title('');
xlabel('Time [s]');
ylabel('w_trans_1 [rad/s]')

fig = figure;
plot(T_trans_1);
title('');
xlabel('Time [s]');
ylabel('T_trans_1 [Nm]')

fig = figure;
plot(P_trans_1);
title('');
xlabel('Time [s]');
ylabel('P_trans_1 [W]')

%% Electric Motor
fig = figure;
plot(w_em);
title('');
xlabel('Time [s]');
ylabel('w_em [rad/s]')

fig = figure;
plot(T_em);
title('');
xlabel('Time [s]');
ylabel('T_em [Nm]');

fig = figure;
plot(P_em);
title('');
xlabel('Time [s]');
ylabel('P_em [W]')

%% Electric Generator
fig = figure;
plot(w_eg);
title('');
xlabel('Time [s]');
ylabel('w_eg [rad/s]')

fig = figure;
plot(T_eg);
title('');
xlabel('Time [s]');
ylabel('T_eg [Nm]')

fig = figure;
plot(P_eg);
title('');
xlabel('Time [s]');
ylabel('P_eg [W]')

%% ICE - EG Transmission
fig = figure;
plot(w_trans_2);
title('');
xlabel('Time [s]');
ylabel('w_trans_2 [rad/s]')

fig = figure;
plot(T_trans_2);
title('');
xlabel('Time [s]');
ylabel('T_trans_2 [Nm]')

fig = figure;
plot(P_trans_2);
title('');
xlabel('Time [s]');
ylabel('P_trans_2 [W]')

%% Internal Combustion Engine
fig = figure;
plot(w_ice);
title('');
xlabel('Time [s]');
ylabel('w_ice [rad/s]')

fig = figure;
plot(T_trans_2);
title('');
xlabel('Time [s]');
ylabel('T_ice [Nm]')

fig = figure;
plot(P_trans_2);
title('');
xlabel('Time [s]');
ylabel('P_ice [W]')


%% Fuel Tank
fig = figure;
plot(m_dot_fuel);
title('');
xlabel('Time [s]');
ylabel('m_dot_fuel [kg/s]')

fig = figure;
plot(m_fuel);
title('');
xlabel('Time [s]');
ylabel('m_fuel [kg]')

%% Battery
fig = figure;
plot(P_b);
title('');
xlabel('Time [s]');
ylabel('P_b [W]')

fig = figure;
plot(q_b);
title('');
xlabel('Time [s]');
ylabel('q_b [%]')

fig = figure;
plot(Q_b);
title('');
xlabel('Time [s]');
ylabel('Q_b [Ah]')

fig = figure;
plot(U_b);
title('');
xlabel('Time [s]');
ylabel('U_b [V]')

fig = figure;
plot(I_b);
title('');
xlabel('Time [s]');
ylabel('I_b [A]')

