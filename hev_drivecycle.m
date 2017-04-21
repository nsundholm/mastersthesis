%% Speed profile generation
% x = [0 20 40 60 80 100 120];
% y = [0 30 50 50 40 20 20]/3.6;
% t_ref = linspace(min(x), max(x), (max(x)-min(x)) + 1);
% v_ref = spline(x,y,t_ref)';

%% Running data
% t_ref = xlsread('rundata.xlsx',1,'B:B');
% s_ref = xlsread('rundata.xlsx',1,'G:G');
% v_ref = xlsread('rundata.xlsx',1,'H:H');
% notch = xlsread('rundata.xlsx',1,'AB:AB');
% 
% tmin = floor(min(t_ref));
% tmax = floor(max(t_ref));
% 
% srefts = timeseries(s_ref,t_ref);
% srefts = resample(srefts,tmin:tmax);
% s_tref = srefts.time;
% s_ref = srefts.data*1000;
% s_tref = [s_tref' (s_tref(end)+1):s_tref(end)+100];
% s_ref = [s_ref' repmat(s_ref(end),1,100)];
% smax = max(s_ref);
% figure;
% plot(s_tref,s_ref);
% xlabel('Time [s]');
% ylabel('s_{ref} [m]');
% 
% vrefts = timeseries(v_ref,t_ref);
% vrefts = resample(vrefts,tmin:tmax);
% v_tref = vrefts.time;
% v_ref = vrefts.data/3.6;
% v_tref = [v_tref' (v_tref(end)+1):v_tref(end)+100];
% v_ref = [v_ref' repmat(v_ref(end),1,100)];
% figure;
% plot(v_tref,v_ref*3.6);
% xlabel('Time [s]');
% ylabel('v_{ref} [km/h]');
% 
% notchts = timeseries(notch,t_ref);
% notchts = resample(notchts,tmin:tmax);
% notch_tref = notchts.time;
% notch_ref = round(notchts.data);
% notch_tref = [notch_tref' (notch_tref(end)+1):notch_tref(end)+100];
% notch_ref = [notch_ref' repmat(0,1,100)];
% figure;
% stairs(notch_tref,notch_ref);
% xlabel('Time [s]');
% ylabel('Notch');

%% Generated speed profile

notch_ref = [repmat(1,1,70) repmat(0,1,30) repmat(1,1,10) repmat(0,1,30) repmat(1,1,10) repmat(0,1,30) repmat(1,1,10) repmat(0,1,30) repmat(-1,1,40) repmat(0,1,100)];
v_ref = zeros(1,length(notch_ref));

for k=1:length(notch_ref)
    switch notch_ref(k)
        case 1
            Ft = Ft_max(round(v_ref(k)*3.6)+1);
            Fmb = 0;
        case 0
            Ft = 0;
            Fmb = 0;
        case -1
            Ft = Ft_min(round(v_ref(k)*3.6)+1);
            Fmb = Ft - Ft_min(1); 
        otherwise
            error('Invalid runmode!');
    end
    a = (Ft - Fmb - (r2*v_ref(k)^2 + r1*v_ref(k) + r0))/m;
    v_ref(k+1) = v_ref(k) + a;
    if v_ref(k+1) < 0
        v_ref(k+1) = 0;
        break;
    end
end

plot(0:1:(length(v_ref)-1),3.6*v_ref)
xlabel('Time [s]')
ylabel('v [km/h]')




