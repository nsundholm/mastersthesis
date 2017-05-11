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
end_time = '260';
solver_type = 'Fixed-step';
Ts = 0.1;

% ********************

run('hev_parameters')
run('hev_drivecycle')
load('PWAM.mat')

% Simulink setup
hevmodel = 'hev_simulation';
open_system(hevmodel);
cset = getActiveConfigSet(hevmodel);
cset.set_param('StartTime',start_time);
cset.set_param('StopTime',end_time);
cset.set_param('SolverType',solver_type);
cset.set_param('FixedStep',sprintf('%f',Ts));

% Run simulation
if run_from_script == 1
    tic;sim(hevmodel);toc;
end

%% Plot simulated data

simdata = struct('data',{{Ft},{Fmb},{Pes},{s},{vref,v},{Pe},{q},{Pb},{mf}},...
                 'plottype',{{2},{2},{2},{1},{1},{1},{1},{1},{1}},...
                 'scale',{{0.001},{0.001},{0.001},{1},{3.6},{0.001},{1},{0.001},{0.001}},...   
                 'legend',{{'tractive force'},{'mechanical break force'},{'engine reference power'},{'position'},{'simulated speed'},{'engine power'},{'SoC'},{'battery power'},{'fuel mass'}},...
                 'title',{{'Tractive Force'},{'Mechanical Break Force'},{'Engine Reference Power'},{'Position'},{'Speed'},{'Engine Power'},{'SoC'},{'Battery Power'},{'Fuel Consumption'}},...
                 'filename',{{'Ft'},{'Fmb'},{'Pices'},{'s'},{'v'},{'Pice'},{'q'},{'Pb'},{'mf'}},...
                 'xlabel',{'t [s]'},...
                 'ylabel',{{'F_t [kN]'},{'F_{mb} [kN]'},{'P_{ice}^* [kN]'},{'s [m]'},{'v [km/h]'},{'P_{ice} [kW]'},{'q [-]'},{'P_b [kW]'},{'m_f [kg]'}});

% Plot simulated data    
if plotting == 1
    % Create directory if plots should be saved
    if saveplotting == 1
        date = datestr(now,'mm-dd HH:MM:SS');
        mkdir('Figures',date);x
    end
    
    for i = 1:length(simdata)
        filename = char(simdata(i).filename);
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
        xlim([0 260]);
        grid on;
        
        if saveplotting == 1
            saveas(gcf,fullfile(sprintf('Figures/%s',date),sprintf('%s.jpg',filename)));%,'epsc2');
        end
    end
end
