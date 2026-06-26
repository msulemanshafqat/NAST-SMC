% Experiment_No3_Robustness_to_Uncertainties_Disturbances_Noise.m
% Experiment No 3: Robustness to Uncertainties, Disturbances, and Noise
% Tests with parameter variations, sigma_f/sigma_tau, and Gaussian noise.

%% Initialization as a separate m file
% opt=false;
if ~exist("opt", "var") || ~opt 
    Initialization
    % Few common changes, without finding them in Initialization
    vis=true;
    save_vis=false;
    save_metrics=true;
    % All possible techs (commented out for reference)
    % alltech:{'NASTSMC', 'FracFITSMC', 'NSTSTSMC', 'CABDISTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};  %
    % Selected Techniques
    % tech={'NASTSMC', 'FracFITSMC', 'CABDISTSMC', 'NSTSTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC', 'PD'};
    % tech={'NASTSMC'};
    % tech={'NASTSMC', 'FracFITSMC','STSMC', 'DISMC', 'ISMC', 'PD'};
    tech={'ETNASTSMC', 'NASTSMC'};    

end
exp_name='exp3';
thisexp_tests={'uncertainty', 'winddist', 'torquedist', 'noise'};
sim_params.test = thisexp_tests; % for display in visualization comparison
results.(exp_name) = struct();
testnum=1; % Spiral only     % alltest:{'spiral', 'circle', 'square', 'random', 'unit'};
sim_params.testnum=testnum;
params.m_orig = params.m;
params.J_orig = params.J;        

% Initialize table for metrics data append for each test during experiment 
compile_results_init

%% Main Loop for Control Technique
for technum=1:length(tech)
    tech_name=tech{technum};
    sim_params.techname=tech_name;
    results.(exp_name).(tech_name) = struct();
    %% Tests
    for thisexp_testnum=1:length(thisexp_tests)
        test_name=thisexp_tests{thisexp_testnum};
        results.(exp_name).(tech_name).(test_name) = struct();
        %% === EXP3 — ROBUSTNESS TESTS (realistic & discriminative) ===
        uncertainty = 0;                     % +50% mass, +50% inertia (battery + payload variation)
        params.m = params.m_orig * (1 + uncertainty);
        params.J = params.J_orig * (1 + uncertainty);
        sigma_f   = zeros(3,1);
        sigma_tau = zeros(3,1);
        noise_var = 0;
        switch test_name
            
            case 'uncertainty'  % Parameter uncertainty — strong but realistic
                uncertainty = 0.5;                     % +50% mass, +50% inertia (battery + payload variation)
                params.m = params.m_orig * (1 + uncertainty);
                params.J = params.J_orig * (1 + uncertainty);
            
            case 'winddist'  % Wind disturbance — 8–12 m/s constant + gusts (urban/outdoor flight)
                sigma_f = @(t) [8; 4; 0] ...           % constant 8–12 m/s wind
                          + 4*[sin(2*t); cos(2*t); 0].*(rand > 0.7);  % occasional gusts        
            
            case 'torquedist'  % Aggressive torque disturbance (e.g. slung load, collision recovery)
                sigma_tau = @(t) 2.5*[sin(3*t); cos(3*t); sin(2*t)];  % peak ≈5–6 Nm (very strong!)

            case 'noise'  % High sensor noise (low-cost IMU + GPS dropout simulation)
                noise_var = 0.08;                      % ≈ ±0.28 rad (~16°) attitude noise + 0.5 m/s velocity noise
        end
                
        %% Simulation, fixed time based integration
        [x_out, x_d_out, u_out, diverged] = octrotor_dynamics(zeta_d_func{testnum}, v_d_func{testnum}, yaw_d_func{testnum}, params, gains, sim_params, 0, 0, [], sigma_f, sigma_tau, noise_var); % no delay, fault_percent, fault_rotors.
    
        if diverged
            break
        end

        %% Store and computing metrics as separate file for commonality among experiments
        compile_results

    end

    if diverged
        break
    end
end
if length(tech)>1, RankMetrics; end
metrics_table_exp3 = metrics_table;
%% Saving and displaying results
if ~opt
    % Metrics Display
    disp('Experiment 3 completed successfully.');
    disp('Metrics Table for Exp3:');
    disp(metrics_table_exp3);
    if ~exist('date_str','var')
        date_str =  string(datetime('now','Format','yMMMd_HHmm'));
    end
    if save_metrics
        % Exporting the metrics table to an Excel file
        writetable(metrics_table_exp3, strcat(save_loc, 'Exp3_Metrics_', tech_name, '_', date_str, '.xlsx'));
        
        % Saving results structure to .mat
        save(strcat(save_loc, 'Exp3_Uncert_Dist_Noise_', date_str,'.mat'), 'results', 'params', 'gains', 'sim_params', 'metrics_table_exp3', 'exp_name', 'tech');
        disp(strcat('Metrics data saved to ', save_loc));
    else
        disp('Metrics not saved');
    end
end

% Executing Visuatization for Experiment No 1
if ~opt && vis 
    if isscalar(tech)
        Visualization;  % Simple visualization for single control technique
    elseif length(tech) > 1   
        Visualization_Comparison;  % Executing Visuatization for Comparison of Techniques
    else
        disp('Visualization Failed.');
    end
end
