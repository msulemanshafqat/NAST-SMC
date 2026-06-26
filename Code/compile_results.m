% compile_results.m
% Store and computing metrics as separate file for commonality among experiments

%% Computing state errors for metrics
zeta_d = x_d_out(:,1:3);
e_zeta = x_d_out(:,1:3) - x_out(:,1:3);
e_eta = x_d_out(:,7:9) - x_out(:,7:9);

t_out = sim_params.tspan(1:size(x_out,1));

% Extracting Power Consumed due to control effort
power_consumed_t = u_out(:,1);

%% Computing metrics and storing
results.(exp_name).(tech_name).(test_name) = struct();
results.(exp_name).(tech_name).(test_name).t = t_out;
results.(exp_name).(tech_name).(test_name).x = x_out;
results.(exp_name).(tech_name).(test_name).x_d = x_d_out;
results.(exp_name).(tech_name).(test_name).u = u_out;


%% Position
% Per-axis RMSE Position
rmse_pos_x = sqrt(mean(e_zeta(:,1).^2));
rmse_pos_y = sqrt(mean(e_zeta(:,2).^2));
rmse_pos_z = sqrt(mean(e_zeta(:,3).^2));

% Mean of RMSE Position
results.(exp_name).(tech_name).(test_name).rmse_pos = mean([rmse_pos_x rmse_pos_y rmse_pos_z]);

% Normalize by trajectory length
traj_length = trapz(t_out, sqrt(sum(x_d_out(:,1:3).^2, 2)));  % Integral \|v_d\|
rmse_pos_norm = results.(exp_name).(tech_name).(test_name).rmse_pos / max(traj_length, 1e-3);  % Avoid divide by zero


% Additional Error Metrics for Position
% Per-axis computations then mean over axis

% SMAPE (Symmetric Mean Absolute Percentage Error): mean over axes
smape_x = mean( abs(e_zeta(:,1)) ./ max( (abs(zeta_d(:,1)) + abs(x_out(:,1))) / 2, 1e-6) * 100 );  % Avoid div0
smape_y = mean( abs(e_zeta(:,2)) ./ max( (abs(zeta_d(:,2)) + abs(x_out(:,2))) / 2, 1e-6) * 100 );
smape_z = mean( abs(e_zeta(:,3)) ./ max( (abs(zeta_d(:,3)) + abs(x_out(:,3))) / 2, 1e-6) * 100 );
results.(exp_name).(tech_name).(test_name).SMAPE = mean([smape_x smape_y smape_z]);  % Overall SMAPE (%)

% ISE (Integral Squared Error): trapz over time, then mean over axes
ise_x = trapz(t_out, e_zeta(:,1).^2);
ise_y = trapz(t_out, e_zeta(:,2).^2);
ise_z = trapz(t_out, e_zeta(:,3).^2);
results.(exp_name).(tech_name).(test_name).ISE = mean([ise_x ise_y ise_z]);

% IAE (Integral Absolute Error): trapz over time, then mean over axes
iae_x = trapz(t_out, abs(e_zeta(:,1)));
iae_y = trapz(t_out, abs(e_zeta(:,2)));
iae_z = trapz(t_out, abs(e_zeta(:,3)));
results.(exp_name).(tech_name).(test_name).IAE = mean([iae_x iae_y iae_z]);

% ITAE (Integral Time-weighted Absolute Error): trapz over time, then mean over axes
itae_x = trapz(t_out, t_out' .* abs(e_zeta(:,1)));
itae_y = trapz(t_out, t_out' .* abs(e_zeta(:,2)));
itae_z = trapz(t_out, t_out' .* abs(e_zeta(:,3)));
results.(exp_name).(tech_name).(test_name).ITAE = mean([itae_x itae_y itae_z]);

% ITSE (Integral Time-weighted Squared Error): trapz over time, then mean over axes
itse_x = trapz(t_out, t_out' .* e_zeta(:,1).^2);
itse_y = trapz(t_out, t_out' .* e_zeta(:,2).^2);
itse_z = trapz(t_out, t_out' .* e_zeta(:,3).^2);
results.(exp_name).(tech_name).(test_name).ITSE = mean([itse_x itse_y itse_z]);


%% Attitude
% Per-axis RMSE Attitude
rmse_att_roll  = sqrt(mean(e_eta(:,1).^2));
rmse_att_pitch = sqrt(mean(e_eta(:,2).^2));
rmse_att_yaw   = sqrt(mean(e_eta(:,3).^2));

% Mean of RMSE Attitude
results.(exp_name).(tech_name).(test_name).rmse_att = mean([rmse_att_roll rmse_att_pitch rmse_att_yaw]);  % vector, or mean for scalar

%% Settling Time and Overshoot
% results.(exp_name).(tech_name).(test_name).settling_time = find_settling_time(e_zeta, 0.02, sim_params.dt);
results.(exp_name).(tech_name).(test_name).overshoot = max(abs(e_zeta)) / norm(results.(exp_name).(tech_name).(test_name).x_d(end,1:3) - sim_params.x0(1:3));

%% Power and Energy Consumed
results.(exp_name).(tech_name).(test_name).energy = trapz(t_out, power_consumed_t)/3600; %Wh

%% Append metrics table data (for display or export)

% row starts for 1 to number of tests in the experiment
metrics_table.Tech(row) = string(tech_name);
metrics_table.Test(row) = string(test_name);

wind_dist = sigma_f(1);
if ~wind_dist(1)==0
    metrics_table.Wind_Dist(row)   = mat2str('8–12 m/s + gusts');            
else
    metrics_table.Wind_Dist(row)   = mat2str(0); 
end

torque_dist = sigma_tau(1);
if ~torque_dist(1)==0
    metrics_table.Torque_Dist(row) = mat2str('sinusoidal with 5–6 Nm peak'); 
else
    metrics_table.Torque_Dist(row) = mat2str(0); 
end

metrics_table.Fault_Rotors(row) = mat2str(fault_rotors);
metrics_table.Fault_Pct(row) = fault_pct;
metrics_table.Delta_ms(row) = delta*1000;
metrics_table.Uncertainty(row) = uncertainty;
metrics_table.Noise_Var(row) = noise_var;

metrics_table.RMSE_Pos_X(row) = rmse_pos_x;
metrics_table.RMSE_Pos_Y(row) = rmse_pos_y;
metrics_table.RMSE_Pos_Z(row) = rmse_pos_z;
metrics_table.RMSE_Pos(row) = results.(exp_name).(tech_name).(test_name).rmse_pos; % final RMSE
metrics_table.RMSE_Pos_TrajNormalized(row) = rmse_pos_norm;

metrics_table.RMSE_Att_Yaw(row) = rmse_att_yaw;
metrics_table.RMSE_Att(row) = results.(exp_name).(tech_name).(test_name).rmse_att;

metrics_table.Settling_Time(row) = 0; %results.(exp_name).(tech_name).(test_name).settling_time;
metrics_table.Overshoot_X(row) = results.(exp_name).(tech_name).(test_name).overshoot(1);
metrics_table.Overshoot_Y(row) = results.(exp_name).(tech_name).(test_name).overshoot(2);
metrics_table.Overshoot_Z(row) = results.(exp_name).(tech_name).(test_name).overshoot(3);

metrics_table.Energy_Consumed_Wh(row) = results.(exp_name).(tech_name).(test_name).energy;

metrics_table.SMAPE(row) = results.(exp_name).(tech_name).(test_name).SMAPE;
metrics_table.ISE(row) = results.(exp_name).(tech_name).(test_name).ISE;
metrics_table.IAE(row) = results.(exp_name).(tech_name).(test_name).IAE;
metrics_table.ITAE(row) = results.(exp_name).(tech_name).(test_name).ITAE;
metrics_table.ITSE(row) = results.(exp_name).(tech_name).(test_name).ITSE;


metrics_table.Score(row) = 0; % Number of indices where the controller ranks 1st (out of all tech) for each test - initiating with zero

% Increment Row for next test data append in next row of table
row=row+1;