close all;
clearvars;

% ********************
% Simulation settings
% ********************

start_from_simulink = 1;
start_time = '0';
end_time = '800';
solver_type = 'Fixed-step';
Ts = 1;
plotting = 1;

% ********************




% Simulink setup
hevmodel = 'hev_simulation';
open_system(hevmodel);
cset = getActiveConfigSet(hevmodel);
cset.set_param('StartTime',start_time);
cset.set_param('StopTime',end_time);
cset.set_param('SolverType',solver_type);
cset.set_param('FixedStep',sprintf('%f',Ts));

% Load speed profile
run('hev_drivecycle');

% Load HEV models
run('hev_models');

% Run simulation
if start_from_simulink ~= 1
    sim(hevmodel)
end

%% Plot simulated data
simdata = struct('data',{{vref,v},{q},{Ft},{Peng},{Pb}},...
                 'plottype',{{1},{1},{1},{2},{1}},...
                 'scale',{{3.6},{1},{0.001},{0.001},{0.001}},...   
                 'legend',{{'reference speed','simulated speed'},{'SoC'},{'tractive force'},{'engine power'},{'battery power'}},...
                 'title',{{'Vehicle Speed'},{'Battery SoC'},{'Tractive Force'},{'Engine Power'},{'Battery Power'}},...
                 'xlabel',{'t [s]'},...
                 'ylabel',{{'v [km/h]'},{'SoC []'},{'F_t [kN]'},{'P_{eng} [kW]'},{'P_b [kW]'}});

if plotting == 1
    for i = 1:length(simdata)
        filename = [char(simdata(i).title),' ',datestr(now, 'dd-mm-yy')];
        figure('name',filename,'numbertitle','off');
        for j = 1:length(simdata(i).data)
            if cell2mat(simdata(i).plottype) == 1
                plot(tout, simdata(i).data{j}.signals.values*cell2mat(simdata(i).scale),'LineWidth',1.5);
            else
                stairs(tout, simdata(i).data{j}.signals.values*cell2mat(simdata(i).scale),'LineWidth',1.5);
            end
            hold on;
        end
        title(simdata(i).title);
        legend(simdata(i).legend);
        xlabel(simdata(i).xlabel);
        ylabel(simdata(i).ylabel);
        grid on;
    end
end