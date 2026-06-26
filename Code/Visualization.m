% Visualization_Exp1_Nominal_Tracking.m
% Visualization script for Experiment No 1: Nominal Reference Tracking
% Loads .mat from Exp1, visualizes trajectories, errors, and metrics.
% Uses structures from revised code. Generates figures for position, attitude, errors, and 3D paths.
% Saves .fig and high-res .png with timestamps.

% Set defaults for IEEE-style figures
set(0,'DefaultAxesFontSize',8); % Eight point Times is suitable typeface for an IEEE paper. Same as figure caption size
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex') % Allows us to use LaTeX maths notation
set(0, 'DefaultAxesFontName', 'times');

% Consistent colors
colors.pos_x = [0.8660 0.3290 0.0000]; % red for x
colors.pos_y = [0.0660 0.4430 0.7450]; % blue for y
colors.pos_z = [0.9290 0.6940 0.1250]; % yellow for z
colors.att_roll = colors.pos_y; % blue for roll (corresponding to y movement)
colors.att_pitch = colors.pos_x; % red for pitch (corresponding to x movement)
colors.att_yaw = [0.2310 0.6660 0.1960]; % green for yaw
colors.error = [0.5 0.5 0.5]; % gray for errors

% Load data (if workspace is clear) otherwise use current Exp data
if ~exist('results','var')
    date_str = '2026Jan18_1637'; % placeholder; replace with actual or dir to fin
    load(['D:\ASNSTSTSMC_2025\Final Results for Paper\Comparison' '\Exp1_Nominal_Tracking_' date_str '.mat'], 'results', 'params', 'gains', 'sim_params', 'metrics_table', 'exp_name', 'tech');
    % load([pwd '\Results\' 'Exp1_Nominal_Tracking_' date_str '.mat'], 'results', 'params', 'gains', 'sim_params', 'metrics_table', 'exp_name', 'tech');
    % load([pwd '\Results\' 'Exp2_Delays_Faults_' date_str '.mat'], 'results', 'params', 'gains', 'sim_params_exp2', 'metrics_table_exp2', 'exp_name', 'tech');
end

for j = 1:length(sim_params.test)
    i = j; % i is a variable to distribute figure positions around the screen for better visibility
    test_name = sim_params.test{j};
    t = results.(exp_name).(tech_name).(test_name).t;
    x = results.(exp_name).(tech_name).(test_name).x;
    x_d = results.(exp_name).(tech_name).(test_name).x_d;
    power_consumed = results.(exp_name).(tech_name).(test_name).u(:,1);
    u_tau = results.(exp_name).(tech_name).(test_name).u(:,2:4);
    u_f_z = results.(exp_name).(tech_name).(test_name).u(:,5);
    f_u_z = results.(exp_name).(tech_name).(test_name).u(:,6);
    tau_u = results.(exp_name).(tech_name).(test_name).u(:,7:9);
    if j==5, i=3.5; end % to keep the lately added unit test figures within the screen without changing my figures positions logic
    %% Velocity figure for this tests
    fig_vel.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[1 2+(i-1)*6 8.89 4.2]);
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    hold on;
    plot(t, x(:,4), 'Color', colors.pos_x, 'LineWidth', 1.5); % x
    plot(t, x(:,5), 'Color', colors.pos_y, 'LineWidth', 1.5); % y
    plot(t, x(:,6), 'Color', colors.pos_z, 'LineWidth', 1.5); % z
    plot(t, x_d(:,4), '--', 'Color', colors.pos_x, 'LineWidth', 1); % x_d
    plot(t, x_d(:,5), '--', 'Color', colors.pos_y, 'LineWidth', 1); % y_d
    plot(t, x_d(:,6), '--', 'Color', colors.pos_z, 'LineWidth', 1); % z_d
    % axis([t(1) t(end) -3 3])
    % set(gca,'YTick',[-3, -2, -1, 0, 1, 2, 3]) %Imposing sensible tickmark locations
    % set(gca,'YTickLabel',{'-3', '-2 ', '-1 ', '0', '1 ', '2 ', '3 '}) %informative labels at the tickmarks assuming -5 and +5 are positional limits
    set(gca, 'TickDir', 'out');
    box off %Removes the borders of the plot area
    xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    ylabel({'↑'; 'Vel'; '(m/s)'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    title(['Velocity for Test: ' test_name], 'Interpreter', 'tex');
    legend('x', 'y', 'z', 'Location', 'best');    
    if save_vis
        savefig(strcat(save_loc, 'Fig_', exp_name, '_',test_name,'_Vel_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Vel_',date_str,'.png'), '-m3', '-nocrop');
    end

%% Position figure for this tests
    fig_pos.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[1 2+(i-1)*6 8.89 4.2]);
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    hold on;
    plot(t, x(:,1), 'Color', colors.pos_x, 'LineWidth', 1.5); % x
    plot(t, x(:,2), 'Color', colors.pos_y, 'LineWidth', 1.5); % y
    plot(t, x(:,3), 'Color', colors.pos_z, 'LineWidth', 1.5); % z
    plot(t, x_d(:,1), '--', 'Color', colors.pos_x, 'LineWidth', 1); % x_d
    plot(t, x_d(:,2), '--', 'Color', colors.pos_y, 'LineWidth', 1); % y_d
    plot(t, x_d(:,3), '--', 'Color', colors.pos_z, 'LineWidth', 1); % z_d
    % axis([t(1) t(end) -5 5])
    % set(gca,'YTick',[-5, -3, -2, -1, 0, 1, 2, 3, 5]) %Imposing sensible tickmark locations
    % set(gca,'YTickLabel',{'\zeta_{min}', '-3 m', '-2 m', '-1 m', '0', '1 m', '2 m', '3 m', '\zeta_{max}'}) %informative labels at the tickmarks assuming -5 and +5 are positional limits
    set(gca, 'TickDir', 'out');
    box off %Removes the borders of the plot area
    xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    ylabel({'↑'; 'Pos'; '(m)'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    title(['Position for Test: ' test_name], 'Interpreter', 'tex');
    legend('x', 'y', 'z', 'Location', 'best');    
    if save_vis
        savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Pos_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Pos_',date_str,'.png'), '-m3', '-nocrop');
    end

    %% Attitude tracking figure for this tests
    fig_att.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[11 2+(i-1)*6 8.89 4.2]);
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    eta_d = x_d(:,7:9)*180/pi; %[roll, pitch, yaw]
    eta = x(:,7:9)*180/pi;
    hold on;
    plot(t, eta(:,1), 'Color', colors.att_roll, 'LineWidth', 1.5); % roll
    plot(t, eta(:,2), 'Color', colors.att_pitch, 'LineWidth', 1.5); % pitch
    plot(t, eta(:,3), 'Color', colors.att_yaw, 'LineWidth', 1.5); % yaw
    plot(t, eta_d(:,1), '--', 'Color', colors.att_roll, 'LineWidth', 1); % desired roll
    plot(t, eta_d(:,2), '--', 'Color', colors.att_pitch, 'LineWidth', 1); % desired pitch
    plot(t, eta_d(:,3), '--', 'Color', colors.att_yaw, 'LineWidth', 1); % desired yaw
    % xlim([t(1) t(end)]);
    % ylim([-60 60]); % limits for attitudes
    % set(gca,'YTick',[-90 -60 -30, 0, 30, 60, 90]) %Imposing sensible tickmark locations
    % set(gca,'YTickLabel',{'\eta_{min}', '-60', '-30', '0', '30', '60', '\eta_{max}'}) %informative labels at the tickmarks assuming -30 and +30 degrees are attitude limits
    set(gca, 'TickDir', 'out');
    box off         %Removes the borders of the plot area
    xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    ylabel({'↑'; 'Attitude'; '(deg)'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % legend('roll', 'pitch', 'yaw', 'roll_d', 'pitch_d', 'yaw_d', 'Location', 'best');
    title(['Attitude for Test: ' test_name], 'Interpreter', 'tex');
    legend('\phi', '\theta', '\psi', '\phi_d', '\theta_d', '\psi_d', 'Location', 'best');
    if save_vis
        savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Att_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Att_',date_str,'.png'), '-m3', '-nocrop');
    end

    %% Positional Error norms for this test
    fig_err.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[21 2+(i-1)*6 8.89 4.2]);
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    e_zeta = x_d(:,1:3) - x(:,1:3);
    e_norm = sqrt(sum(e_zeta.^2, 2));
    plot(t, e_norm, 'Color', colors.error, 'LineWidth', 1.5); % x error
    title(['Positional Error for Test: ' test_name], 'Interpreter', 'tex');
    box off;
    % set(gca,'YTick',[-30, -20, -10, -5, 0, 5, 10, 20, 30]) %Imposing sensible tickmark locations
    % set(gca,'YTickLabel',{'\eta_{min}', '-20', '-10', '-5', '0', '5', '10', '20', '\eta_{max}'}) %informative labels at the tickmarks assuming -30 and +30 degrees are attitude limits
    % xlim([t(1) t(end)]);
    ylim([0 max(e_norm)*1.1]);
    set(gca, 'TickDir', 'out');
    xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    ylabel({'↑'; 'Error'; 'Norm'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    legend('Error', 'Location', 'best');
    if save_vis
        savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Err_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Err_',date_str,'.png'), '-m3', '-nocrop');
    end

    %% Attitude Error norms for this test
    fig_err_att.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[21 2+(i-1)*6 8.89 4.2]);
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    e_eta = x_d(:,7:9) - x(:,7:9);
    e_att_norm = sqrt(sum(e_eta.^2, 2));
    plot(t, e_att_norm, 'Color', colors.error, 'LineWidth', 1.5); % x error
    title(['Attitude Error for Test: ' test_name], 'Interpreter', 'tex');
    box off;
    % set(gca,'YTick',[-30, -20, -10, -5, 0, 5, 10, 20, 30]) %Imposing sensible tickmark locations
    % set(gca,'YTickLabel',{'\eta_{min}', '-20', '-10', '-5', '0', '5', '10', '20', '\eta_{max}'}) %informative labels at the tickmarks assuming -30 and +30 degrees are attitude limits
    % xlim([t(1) t(end)]);
    ylim([0 max(e_att_norm)*1.1]);
    set(gca, 'TickDir', 'out');
    xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    ylabel({'↑'; 'Error'; 'Norm'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['Attitude Error for Test: ' test_name], 'Interpreter', 'tex');
    legend('Error', 'Location', 'best');
    if save_vis
        savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_ErrAtt_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_ErrAtt_',date_str,'.png'), '-m3', '-nocrop');
    end

    %% 3D Trajectories for this test
    fig_3d.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[31 2+(i-1)*6 8.89 6]); % taller for 3D
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 4.9]);
    hold on;
    plot3(x(:,1), x(:,2), x(:,3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % actual
    plot3(x_d(:,1), x_d(:,2), x_d(:,3), '--r', 'LineWidth', 1); % desired
    xlabel('x (m)', 'FontWeight', 'bold');
    ylabel('y (m)', 'FontWeight', 'bold');
    zlabel('z (m)', 'FontWeight', 'bold');
    legend('Actual', 'Desired', 'Location', 'best');
    grid on; view(45, 30);
    box off;
    axis tight; % or set limits e.g., xlim([-3 3]); ylim([-3 3]); zlim([0 3]);
    set(gca, 'TickDir', 'out');
    if save_vis
        savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_3DTraj_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_3DTraj_',date_str,'.png'), '-m3', '-nocrop');
    end
    
    %% Outer Loop 'Control Input u_f' to the Inner loop and Inner Loop 'Control Input u_tau' to Plant for this test
    % fig_u_f_z.(test_name) = figure;
    % set(gcf, 'Units','centimeters', 'Position',[21 2+(i-1)*6 8.89 4.2]);
    % set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    % hold on;
    % plot(t, u_f_z, '--', 'Color', colors.pos_z, 'LineWidth', 1.5); % u_f_z
    % plot(t, f_u_z, 'Color', colors.pos_z, 'LineWidth', 1.5); % f_u_z
    % 
    % axis tight;
    % % set(gca,'YTick',[-5, -3, -2, -1, 0, 1, 2, 3, 5]) %Imposing sensible tickmark locations
    % % set(gca,'YTickLabel',{'\zeta_{min}', '-3 m', '-2 m', '-1 m', '0', '1 m', '2 m', '3 m', '\zeta_{max}'}) %informative labels at the tickmarks assuming -5 and +5 are positional limits
    % % set(gca, 'TickDir', 'out');
    % box off %Removes the borders of the plot area
    % xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    % ylabel({'↑'; 'u'; ''},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['Test: ' test_name], 'Interpreter', 'tex');
    % legend('u_f_z', 'f_u_z', 'Location', 'best');    
    % if save_vis
    %     savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_fu_z_',date_str,'.fig'));
    %     export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_fu_z_',date_str,'.png'), '-m3', '-nocrop');
    % end


    %% Outer Loop Actual f_u' tau_u' to Multirotor Body for this test
    % fig_tau_u.(test_name) = figure;
    % set(gcf, 'Units','centimeters', 'Position',[31 2+(i-1)*6 8.89 4.2]);
    % set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    % hold on;
    % plot(t, u_tau(:,1), '--', 'Color', colors.att_roll, 'LineWidth', 1); % u_tau_phi
    % plot(t, u_tau(:,2), '--', 'Color', colors.att_pitch, 'LineWidth', 1); % u_tau_theta
    % plot(t, u_tau(:,3), '--', 'Color', colors.att_yaw, 'LineWidth', 1); % u_tau_psi
    % 
    % plot(t, tau_u(:,1), 'Color', colors.att_roll, 'LineWidth', 1.5); % tau_u_phi
    % plot(t, tau_u(:,2), 'Color', colors.att_pitch, 'LineWidth', 1.5); % tau_u_theta
    % plot(t, tau_u(:,3), 'Color', colors.att_yaw, 'LineWidth', 1.5); % tau_u_psi
    % axis tight;
    % % set(gca,'YTick',[-5, -3, -2, -1, 0, 1, 2, 3, 5]) %Imposing sensible tickmark locations
    % % set(gca,'YTickLabel',{'\zeta_{min}', '-3 m', '-2 m', '-1 m', '0', '1 m', '2 m', '3 m', '\zeta_{max}'}) %informative labels at the tickmarks assuming -5 and +5 are positional limits
    % % set(gca, 'TickDir', 'out');
    % box off %Removes the borders of the plot area
    % xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    % ylabel({'↑'; 'u'; ''},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['Test: ' test_name], 'Interpreter', 'tex');
    % legend('u_tau\phi', 'u_tau\theta', 'u_tau\psi', 'tau_u\phi', 'tau_u\theta', 'tau_u\psi', 'Location', 'best');    
    % if save_vis
    %     savefig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Pos_',date_str,'.fig'));
    %     export_fig(strcat(save_loc, 'Fig_',exp_name, '_',test_name,'_Pos_',date_str,'.png'), '-m3', '-nocrop');
    % end

    %% Unit step error zoom (if test_name=='unit_step')
    % if strcmp(test_name, 'unit_step')
    %     fig_step_zoom = figure;
    %     set(gcf, 'Units','centimeters', 'Position',[21 2+(i-1)*6 8.89 4.2]);
    %     set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    %     e_z = results.exp1.unit_step.x_d(:,3) - results.exp1.unit_step.x(:,3);  % z error
    %     plot(results.exp1.unit_step.t, e_z, 'Color', colors.pos_z, 'LineWidth',1.5);
    %     xline(1, '--k', 'Step Time');  % Mark t=1s
    %     box off; xlabel('Time (s)'); ylabel('z Error (m)');
    %     title('Unit Step z Response'); xlim([0 5]); ylim([-0.2 1.2]); grid on;
    %     if save_vis
    %         savefig(strcat(save_loc, 'Fig_Exp1_UnitStep_Zoom_', date_str, '.fig'));
    %         export_fig(strcat(save_loc, 'Fig_Exp1_UnitStep_Zoom_', date_str, '.png'), '-m3', '-nocrop');
    %     end
    % end
end
% Save updated results with metrics if needed
disp('Visualization completed.');
if save_vis
    disp(strcat('Figures saved - ',save_loc));
else
    disp('Figures not saved');
end