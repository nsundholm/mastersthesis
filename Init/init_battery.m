%% init_battery.m

% =======
% Battery
% =======



% #####################################################################################################################

% Global variables
% ----------------
    global h                                    % Stepsize [s] from block "Driving Cycle"
    global Q_BT_IC                              % Initial charge [C] from block "Battery"

% #####################################################################################################################

% Battery parameters
% ------------------
    c_b_E1     =  115;							
    c_b_E2     =  11.75;						
    c_b_E3     =  15;							         			
    c_b_E4     =  4.06;							
    c_b_L1     =  c_b_E1;                       % c_BT_L1 = c_BT_EL1 for continuity reasons
    c_b_L2     =  18.75;					
    c_b_L3     =  c_b_E3;                       % c_BT_L3 = c_BT_EL3 for continuity reasons
    c_b_L4     =  0.00;

% #####################################################################################################################

% Prepare data
% ------------
    Q_b_0      = I_0 * 3600;                   % Discharge current in 1 h              [C]
    U_b_0      = c_b_E1+c_b_E3;                % Mean battery voltage                  [V]
    I_b_max    = (60/t_ch)*I_0;                % Maximum charge/discharge current      [A]
    Q_b_IC     = (Q_b_IC_rel/100)*3600*I_0;    % Initial battery charge                [C]

% #####################################################################################################################
