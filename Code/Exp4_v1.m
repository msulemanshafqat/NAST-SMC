% Experiment No 4: Robustness to Actuator Delays
% This script simulates robustness tests for delays using various trajectories.
% Builds on Exp1 structure: Uses Initialization.m, calls octrotor_dynamics_v2, computes metrics.
% Tests: Varying delays (5-50 ms) across spiral/circle/square/random.
% Save .mat with timestamp; optional visualization.
% Run for different techniques in loop and visualize comparison
% Results compiling and storing in struct as a separate m file:
% 'compile_results.m to keep standard in all experiments
%% Initialization (loads params, gains, etc.)
if ~exist("opt", "var") || ~opt
    Initialization % standard initializations for each experiment
    % Few common changes, without finding them in Initialization
    vis=true;
    save_vis=false;
    save_metrics=true;
    
    % All possible techs (commented out for reference)
    % alltech={'NASTSMC', 'FracFITSMC', 'CABDISTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};  %

    % Selected Techniques
    % tech={'NASTSMC', 'FracFITSMC', 'NSTSTSMC', 'CABDISTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};
    % tech={'NASTSMC', 'FracFITSMC', 'CABDISTSMC'};
    % tech={'NASTSMC', 'CABDISTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};
    % tech={'NASTSMC'};
    % tech={'NASTSMC', 'FracFITSMC','STSMC', 'DISMC', 'ISMC', 'PD'};
    tech={'ETNASTSMC', 'NASTSMC'};    

end

% some exp4 specific initializations
exp_name='exp4';
results.(exp_name) = struct();

% Delay parameters all
% thisexp_tests={'Normal', 'Delay5ms', 'Delay10ms', 'Delay20ms', 'Delay50ms'};
% delta_exp = [0;5;10;15;25]/1000;            % Convert delta_ms to seconds

% Selective stable only
thisexp_tests={'Delay5ms'};
delta_exp = 5/1000;            % Convert delta_ms to seconds

sim_params.test = thisexp_tests; % for display in visualization comparison

% Initialize table for metrics data append for each test during experiment 
compile_results_init

%% Main Loop for Control Technique
for technum=1:length(tech)
    tech_name=tech{technum};
    sim_params.techname=tech_name;
    results.(exp_name).(tech_name) = struct();
     
    %% Sub-loop for exp4-Tests
    testnum=1;
    sim_params.testnum=testnum;
    for thisexp_testnum=1:length(thisexp_tests)
        test_name=thisexp_tests{thisexp_testnum};
        results.(exp_name).(tech_name).(test_name) = struct();
        delta = delta_exp(thisexp_testnum);

        % Run simulation
        [x_out, x_d_out, u_out, diverged] = octrotor_dynamics(zeta_d_func{testnum}, v_d_func{testnum}, yaw_d_func{testnum}, params, gains, sim_params, delta, 0, [], zeros(3,1), zeros(3,1), 0);  % sigma_f=0, sigma_tau=0, noise=0 for baseline
        
        if diverged
            break
        end

        t_out = sim_params.tspan(1:size(x_out,1));
        % Compute errors
        e_zeta = x_d_out(:,1:3) - x_out(:,1:3);
        e_eta = x_d_out(:,7:9) - x_out(:,7:9);
        
        % Extracting Power Consumed due to control effort
        % power_consumed_t = u_out(:,1);
        
        % Store and computing metrics as separate file for commonality among experiments
        compile_results
    end

    if diverged
        break
    end
end
if length(tech)>1, RankMetrics; end
metrics_table_exp4 = metrics_table;
%% Saving and Displaying Results
if ~opt
    disp('Experiment 4 completed successfully.');
    disp('Metrics Table for Exp4:');
    disp(metrics_table_exp4);
    if ~exist('date_str','var')
        date_str =  string(datetime('now','Format','yMMMd_HHmm'));
    end

    if save_metrics
        writetable(metrics_table_exp4, strcat(save_loc, 'Exp4_Metrics_', date_str, '.xlsx'));
        save(strcat(save_loc, 'Exp4_Delays', date_str, '.mat'), ...
         'results', 'params', 'gains', 'sim_params', 'metrics_table_exp4', 'exp_name', 'tech');
        fprintf('Experiment 4 data saved to %s.\n', save_loc);
    else
        disp('Metrics not saved');
    end
end

% Executing Visuatization for Experiment No 4
if vis && ~opt
    if isscalar(tech)
        Visualization;  % Simple visualization for single control technique
    elseif length(tech) > 1   
        Visualization_Comparison;  % Executing Visuatization for Comparison of Techniques
    else
        disp('Visualization Failed.');
    end
end