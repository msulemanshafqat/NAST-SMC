% Experiment No 1: Nominal Reference Tracking

% This script simulates nominal reference tracking experiment for various trajectories for different control techniques.
% Uses structures for parameters, gains, states, and results.

%% Updates notes
% Experiment_No1_Nominal_Reference_Tracking.m is the initial file
% Experiment_No1_Nominal_Reference_Tracking_v2.m is the correctly
% functioning file with two tests but needs gains adjusting for correct response
% Experiment_No1_Nominal_Reference_Tracking_v3.m is where gains are being adjusted
% V4 is where Octrotor_dynamcis is now a function, not ode
% Initialization shifted to separate m file to ensure repeatability
% V5 can now run for different techniques in loop and visualize comparison

%% Initialization as a separate m file
% opt=false;
if ~exist("opt", "var") || ~opt 
    Initialization
    % Few common changes, without finding them in Initialization
    vis=true;
    save_vis=false;
    save_metrics=true;
    
    % Tech: All implemented techs (commented out for reference)
    % alltech:{'NAST-SMC', 'FracFITSMC', 'NSTSTSMC', 'CABDISTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};  %
    
    % Selected techniques for this simulation:
    % tech={'NASTSMC', 'FracFITSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};
    tech={'ETNASTSMC', 'NASTSMC'};    
    % Tests: alltests for reference:{'spiral', 'circle', 'square', 'random', 'unit'};
    sim_params.test={'spiral', 'circle', 'square', 'random', 'unit'};
    
end
% Initialize table for metrics data append for each test during experiment 
compile_results_init
exp_name='exp1';
results.(exp_name) = struct();
%% Main Loop for Control Technique
for technum=1:length(tech)
    tech_name=tech{technum};
    sim_params.techname=tech_name;
    results.(exp_name).(tech_name) = struct();
    %% Sub-Loop for Tests
    for i = 1:length(sim_params.test)
        test_name=sim_params.test{i};
        results.(exp_name).(tech_name).(test_name) = struct();
        switch test_name
            case 'spiral', testnum=1;
            case 'circle', testnum=2;
            case 'square', testnum=3;
            case 'random', testnum=4;
            case 'unit',   testnum=5;
        end
        sim_params.testnum=testnum;
        % Simulation, fixed time based integration
        [x_out, x_d_out, u_out, diverged] = octrotor_dynamics(zeta_d_func{testnum}, v_d_func{testnum}, yaw_d_func{testnum}, params, gains, sim_params, delta, fault_pct, fault_rotors, sigma_f, sigma_tau, noise_var); % no delay, fault_percent, fault_rotors, sigma_f, sigma_tau, noise_var
        
        if diverged
            break
        end

        % Store / computing metrics and append metrics table data (for display or export)
        % Using separate m file for commonality among experiments
        compile_results
    end

    if diverged
        break
    end
end
if length(tech)>1, RankMetrics; end
metrics_table_exp1 = metrics_table;
%% Saving and displaying results
if ~sim_params.opt
    % Metrics Display
    disp('Experiment 1 completed successfully.');
    disp('Metrics Table for Exp1:');
    disp(metrics_table_exp1);
    if ~exist('date_str','var')
        date_str =  string(datetime('now','Format','yMMMd_HHmm'));
    end
    if save_metrics
        % Exporting the metrics table to an Excel file
        if length(tech)>1
            tech_name = 'alltech';
        end
        writetable(metrics_table_exp1, strcat(save_loc, 'Exp1_Metrics_', tech_name, '_', date_str, '.xlsx'));
        
        % Saving results structure to .mat
        save(strcat(save_loc, 'Exp1_Nominal_Tracking_', date_str,'.mat'), 'results', 'params', 'gains', 'sim_params', 'metrics_table_exp1', 'exp_name', 'tech');
        disp(strcat('Metrics data saved to ', save_loc));
    else
        disp('Metrics not saved');
    end
end

% Executing Visuatization for Experiment No 1
if ~sim_params.opt && vis 
    if isscalar(tech)
        Visualization;  % Simple visualization for single control technique
    elseif length(tech) > 1   
        Visualization_Comparison;  % Executing Visuatization for Comparison of Techniques
    else
        disp('Visualization Failed.');
    end
end