%close all;
clearvars;

% ********************
% Simulation settings
% ********************

controller = 2;
run_from_script = 0;
plotting = 1;
saveplotting = 0;

start_time = '0';
end_time = '100';
solver_type = 'Fixed-step';
Ts = 0.1;

% ********************

% Simulink setup
hevmodel = 'hev_simulation';
open_system(hevmodel);
cset = getActiveConfigSet(hevmodel);
cset.set_param('StartTime',start_time);
cset.set_param('StopTime',end_time);
cset.set_param('SolverType',solver_type);
cset.set_param('FixedStep',sprintf('%f',Ts));
run('hev_parameters')
run('hev_drivecycle')
load('PWAM.mat')

% Run simulation
if run_from_script == 1
    tic;sim(hevmodel);toc;
end

%% Plot simulated data

simdata = struct('data',{{Ft},{Fmb},{Pes},{s},{vref,v},{Pe},{q},{Pb},{mf}},...
                 'plottype',{{1},{1},{2},{1},{1},{1},{1},{1},{1}},...
                 'scale',{{0.001},{0.001},{0.001},{1},{3.6},{0.001},{1},{0.001},{0.001}},...   
                 'legend',{{'tractive force'},{'mechanical break force'},{'engine reference power'},{'simulated position'},{'simulated speed'},{'engine power'},{'SoC'},{'battery power'},{'fuel'}},...
                 'title',{{'Tractive Force'},{'Mechanical Break Force'},{'Engine Reference Power'},{'Position'},{'Speed'},{'Engine Power'},{'Battery SoC'},{'Battery Power'},{'Fuel'}},...
                 'xlabel',{'t [s]'},...
                 'ylabel',{{'F_t [kN]'},{'F_{mb} [kN]'},{'P_e^* [kN]'},{'s [m]'},{'v [km/h]'},{'P_e [kW]'},{'q []'},{'P_b [kW]'},{'m_f [kg]'}});

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