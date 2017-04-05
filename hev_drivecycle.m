%% Speed profile generation
% x = [0 20 40 60 80 100 120];
% y = [0 30 50 50 40 20 20]/3.6;
% t_ref = linspace(min(x), max(x), (max(x)-min(x)) + 1);
% v_ref = spline(x,y,t_ref)';

%% Running data
t_ref = xlsread('rundata.xlsx',1,'B:B');
v_ref = xlsread('rundata.xlsx',1,'H:H');
ts = timeseries(v_ref,t_ref);
tsr = resample(ts,0:1:837);
t_ref = tsr.time;
v_ref = tsr.data/3.6;
plot(t_ref,v_ref*3.6);
xlabel('Time [s]');
ylabel('v_{ref} [km/h]');