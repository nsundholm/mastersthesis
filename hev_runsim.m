close all;
clearvars;

% ********************
% Simulation settings
% ********************

run_from_script = 0;
start_time = '0';
end_time = '200';
solver_type = 'Fixed-step';
Ts = 1;
plotting = 1;
saveplotting = 0;

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
if run_from_script == 1
    tic;sim(hevmodel);toc;
end

%% Plot simulated data

simdata = struct('data',{{s},{vref,v},{q},{Ft},{Peng},{Pb},{mf}},...
                 'plottype',{{1},{1},{1},{1},{2},{1},{1}},...
                 'scale',{{1},{3.6},{1},{0.001},{0.001},{0.001},{0.001}},...   
                 'legend',{{'position'},{'reference speed','simulated speed'},{'SoC'},{'tractive force'},{'engine power'},{'battery power'},{'fuel'}},...
                 'title',{{'Vehicle Position'},{'Vehicle Speed'},{'Battery SoC'},{'Tractive Force'},{'Engine Power'},{'Battery Power'},{'Fuel'}},...
                 'xlabel',{'t [s]'},...
                 'ylabel',{{'s [m]'},{'v [km/h]'},{'SoC []'},{'F_t [kN]'},{'P_{eng} [kW]'},{'P_b [kW]'},{'m_f [kg]'}});

if plotting == 1
    for i = 1:length(simdata)
        filename = [char(simdata(i).title),' ',datestr(now, 'dd-mm-yy')];
        figure('name',filename,'numbertitle','off');
        for j = 1:length(simdata(i).data)
            if cell2mat(simdata(i).plottype) == 1
                plot(tout, simdata(i).data{j}.signals.values*cell2mat(simdata(i).scale),'LineWidth',1.25);
            else
                stairs(tout, simdata(i).data{j}.signals.values*cell2mat(simdata(i).scale),'LineWidth',1);
            end
            hold on;
        end
        title(simdata(i).title);
        legend(simdata(i).legend);
        xlabel(simdata(i).xlabel);
        ylabel(simdata(i).ylabel);
        grid on;
        
        if saveplotting == 1
            print(simdata(i).title,'-depsc')
        end
    end
end