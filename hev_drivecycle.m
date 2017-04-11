%% Speed profile generation
% x = [0 20 40 60 80 100 120];
% y = [0 30 50 50 40 20 20]/3.6;
% t_ref = linspace(min(x), max(x), (max(x)-min(x)) + 1);
% v_ref = spline(x,y,t_ref)';

%% Running data
t_ref = xlsread('rundata.xlsx',1,'B:B');
s_ref = xlsread('rundata.xlsx',1,'G:G');
v_ref = xlsread('rundata.xlsx',1,'H:H');

tmin = floor(min(t_ref));
tmax = floor(max(t_ref));

% srefts = timeseries(s_ref,t_ref);
% srefts = resample(srefts,tmin:tmax);
% t_ref = srefts.time;
% s_ref = srefts.data*1000;
% t_ref = [t_ref' (t_ref(end)+1):t_ref(end)+100];
% s_ref = [s_ref' repmat(s_ref(end),1,100)];
% smax = max(s_ref);
% plot(t_ref,s_ref);
% xlabel('Time [s]');
% ylabel('s_{ref} [m]');

vrefts = timeseries(v_ref,t_ref);
vrefts = resample(vrefts,tmin:tmax);
t_ref = vrefts.time;
v_ref = vrefts.data/3.6;
t_ref = [t_ref' (t_ref(end)+1):t_ref(end)+100];
v_ref = [v_ref' repmat(v_ref(end),1,100)];
% plot(t_ref,v_ref*3.6);
% xlabel('Time [s]');
% ylabel('v_{ref} [km/h]');