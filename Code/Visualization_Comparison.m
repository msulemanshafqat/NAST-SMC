% Visualization_Comparison.m
% Loads .mat from Exp1 or Exp2, visualizes trajectories, errors, and metrics.
% Uses structures from revised code. Generates figures for position, attitude, errors, and 3D paths.
% Saves .fig and high-res .png with timestamps.

% Set defaults for IEEE-style figures
set(0,'DefaultAxesFontSize',8); % Eight point Times is suitable typeface for an IEEE paper. Same as figure caption size
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex') % Allows us to use LaTeX maths notation
set(0, 'DefaultAxesFontName', 'times');

% Techniques for Visualization
% tech={'NASTSMC', 'CABDISTSMC', 'BFSMC', 'STSMC', 'DISMC', 'ISMC'};


% Consistent colors
% colors.Desired = [0.8660 0.3290 0.0000]; % red
colors = [0.0660 0.4430 0.7450]; % blue
colors(2,:)= [0.9290 0.6940 0.1250]; % yellow

% colors.att_roll = colors.pos_y; % blue for roll (corresponding to y movement)
% colors.att_pitch = colors.pos_x; % red for pitch (corresponding to x movement)
% colors.att_yaw = [0.2310 0.6660 0.1960]; % green for yaw
% colors.error = [0.5 0.5 0.5]; % gray for errors

% Load data (if workspace is clear) otherwise use current Exp data
if ~exist('date_str','var')
    date_str = '2026Jan18_1637'; % placeholder; replace with actual or dir to fin
    load(['D:\ASNSTSTSMC_2025\Final Results for Paper\Comparison' '\Exp1_Nominal_Tracking_' date_str '.mat'], 'results', 'params', 'gains', 'sim_params', 'metrics_table', 'exp_name', 'tech');
    % load([pwd '\Results\' 'Exp1_Nominal_Tracking_' date_str '.mat'], 'results', 'params', 'gains', 'sim_params', 'metrics_table', 'exp_name', 'tech');
    % load([pwd '\Results\' 'Exp2_Delays_Faults_' date_str '.mat'], 'results', 'params', 'gains', 'sim_params_exp2', 'metrics_table_exp2', 'exp_name', 'tech');
end

for i = 1:length(sim_params.test)
    test_name = sim_params.test{i};

    for technum=1:length(tech)
        tech_name=tech{technum};
        x.(tech_name) = results.(exp_name).(tech_name).(test_name).x;
        t.(tech_name) = results.(exp_name).(tech_name).(test_name).t;
        x_d.(tech_name) = results.(exp_name).(tech_name).(test_name).x_d;
    end
    % fu = results.(exp_name).(tech_name).(test_name).u(:,1:3);
    % tau_u = results.(exp_name).(tech_name).(test_name).u(:,4:6);
    % power_consumed = results.(exp_name).(tech_name).(test_name).u(:,7);
    %% Position (x-axis only) figure for this test (for debugging only)
    % fig_pos.(test_name) = figure;
    % set(gcf, 'Units','centimeters', 'Position',[1 2+(i-1)*6 8.89 4.2]);
    % set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    % hold on;
    % leg={};
    % for technum=1:length(tech)
    %     tech_name=tech{technum};
    %     plot(t.(tech_name), x.(tech_name)(:,1)); % actual
    %     leg=[leg, tech_name];
    % end
    % plot(t.(tech_name), x_d.(tech_name)(:,1), '--r'); % desired
    % leg = [leg, 'Desired'];
    % legend(leg, 'Location', 'best');
    % % axis([t(1) t(end) -5 5])
    % % set(gca,'YTick',[-5, -3, -2, -1, 0, 1, 2, 3, 5]) %Imposing sensible tickmark locations
    % % set(gca,'YTickLabel',{'-5 m', '-3 m', '-2 m', '-1 m', '0', '1 m', '2 m', '3 m', '5 m'}) %informative labels at the tickmarks assuming -5 and +5 are positional limits
    % set(gca, 'TickDir', 'out');
    % box off %Removes the borders of the plot area
    % xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    % ylabel({'↑'; 'xPos'; '(m)'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['x Position Comparison : ' test_name], 'Interpreter', 'tex');
    % if save_vis
    %     savefig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_Pos_',date_str,'.fig'));
    %     export_fig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_Pos_',date_str,'.png'), '-m3', '-nocrop');
    % end

    % % Figure for zoomed view of x position for final interval
    % fig_xposzoom.(test_name) = figure;
    % set(gcf, 'Units','centimeters', 'Position',[1 2+(i-1)*6 8.89 4.2]); 
    % set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    % hold on;
    % leg={};
    % interval=600;
    % for technum=1:length(tech)
    %     tech_name=tech{technum};
    %     plot(t.(tech_name)(end-interval:end), x.(tech_name)(end-interval:end,1)); % actual
    %     leg=[leg, tech_name];
    % end
    % plot(t.(tech_name)(end-interval:end), x_d.(tech_name)(end-interval:end,1), '--r'); % desired
    % xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    % ylabel({'↑'; 'xPos'; '(m)'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['x Position Comparison : ' test_name], 'Interpreter', 'tex');
    % leg = [leg, 'Desired'];
    % legend(leg, 'Location', 'best');
    % grid on; 
    % box off;
    % axis tight;
    % set(gca, 'TickDir', 'out');
    % if save_vis
    %     savefig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_3DTraj_',date_str,'.fig'));
    %     export_fig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_3DTraj_',date_str,'.png'), '-m3', '-nocrop');
    % end
    
    % % Figure for zoomed view of z position for final interval
    % fig_zposzoom.(test_name) = figure;
    % set(gcf, 'Units','centimeters', 'Position',[11 2+(i-1)*6 8.89 4.2]); 
    % set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    % hold on;
    % leg={};
    % interval=600;
    % for technum=1:length(tech)
    %     tech_name=tech{technum};
    %     plot(t.(tech_name)(end-interval:end), x.(tech_name)(end-interval:end,3)); % actual
    %     leg=[leg, tech_name];
    % end
    % plot(t.(tech_name)(end-interval:end), x_d.(tech_name)(end-interval:end,3), '--r'); % desired
    % leg = [leg, 'Desired'];
    % legend(leg, 'Location', 'best');
    % xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    % ylabel({'↑'; 'zPos'; '(m)'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['z Position Comparison : ' test_name], 'Interpreter', 'tex');
    % grid on; 
    % box off;
    % axis tight;
    % set(gca, 'TickDir', 'out');
    % if save_vis
    %     savefig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_3DTraj_',date_str,'.fig'));
    %     export_fig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_3DTraj_',date_str,'.png'), '-m3', '-nocrop');
    % end
    
    %% Error norms for this test
    fig_err.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[21 2+(i-1)*6 8.89 4.2]);
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    hold on
    leg={};
    for technum=1:length(tech)
        tech_name=tech{technum};
        e_traj = x_d.(tech_name)(:,[1:3,9]) - x.(tech_name)(:,[1:3,9]);
        e_traj_norm = sqrt(sum(e_traj.^2, 2));
        plot(t.(tech_name), e_traj_norm, 'LineWidth', 1.5);%, 'Color', colors(technum,:)); % x error
        leg=[leg, tech_name];
    end
    colororder('gem');
    title(['Trajectory Error: ' test_name], 'Interpreter', 'tex');
    box off;
    % set(gca,'YTick',[-30, -20, -10, -5, 0, 5, 10, 20, 30]) %Imposing sensible tickmark locations
    % set(gca,'YTickLabel',{'\eta_{min}', '-20', '-10', '-5', '0', '5', '10', '20', '\eta_{max}'}) %informative labels at the tickmarks assuming -30 and +30 degrees are attitude limits
    % xlim([t(1) t(end)]);
    % ylim([0 max(e_traj_norm)*1.1]);
    set(gca, 'TickDir', 'out');
    xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    ylabel({'↑'; 'Error'; 'Norm'},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    legend(leg, 'Location', 'best');
    if save_vis
        savefig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_Err_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_Err_',date_str,'.png'), '-m3', '-nocrop');
    end

    %% 3D Trajectories for this test
    fig_3d.(test_name) = figure;
    set(gcf, 'Units','centimeters', 'Position',[31 2+(i-1)*6 8.89 6]); % taller for 3D
    set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 4.9]);
    hold on;
    leg={};
    for technum=1:length(tech)
        tech_name=tech{technum};
        plot3(x.(tech_name)(:,1), x.(tech_name)(:,2), x.(tech_name)(:,3),'LineWidth', 1.5);%,  'Color', colors(technum,:)); % actual
        leg=[leg, tech_name];
    end
    plot3(x_d.(tech_name)(:,1), x_d.(tech_name)(:,2), x_d.(tech_name)(:,3), '--r', 'LineWidth', 1); % desired
    xlabel('x (m)', 'FontWeight', 'bold');
    ylabel('y (m)', 'FontWeight', 'bold');
    zlabel('z (m)', 'FontWeight', 'bold');
    leg = [leg, 'Desired'];
    legend(leg, 'Location', 'best');
    grid on; view(45, 30);
    box off;
    axis tight; % or set limits e.g., xlim([-3 3]); ylim([-3 3]); zlim([0 3]);
    set(gca, 'TickDir', 'out');
    if save_vis
        savefig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_3DTraj_',date_str,'.fig'));
        export_fig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_3DTraj_',date_str,'.png'), '-m3', '-nocrop');
    end
    
    
    %% Outer Loop 'Control Input fu' to the Inner loop and Inner Loop 'Control Input tau_u' to Plant for this test (for debugging only)
    % fig_fu.(test_name) = figure;
    % set(gcf, 'Units','centimeters', 'Position',[31 2+(i-1)*6 8.89 4.2]);
    % set(gca, 'Units','centimeters', 'Position',[2 0.9 6.5 2.9]);
    % hold on;
    % plot(t.(tech_name), fu(:,1), 'Color', colors.pos_x, 'LineWidth', 1.5); % fu_x
    % plot(t.(tech_name), fu(:,2), 'Color', colors.pos_y, 'LineWidth', 1.5); % fu_y
    % plot(t.(tech_name), fu(:,3), 'Color', colors.pos_z, 'LineWidth', 1.5); % fu_z
    % plot(t.(tech_name), tau_u(:,1), '--', 'Color', colors.att_roll, 'LineWidth', 1); % tau_u_phi
    % plot(t.(tech_name), tau_u(:,2), '--', 'Color', colors.att_pitch, 'LineWidth', 1); % tau_u_theta
    % plot(t.(tech_name), tau_u(:,3), '--', 'Color', colors.att_yaw, 'LineWidth', 1); % tau_u_psi
    % axis tight;
    % % set(gca,'YTick',[-5, -3, -2, -1, 0, 1, 2, 3, 5]) %Imposing sensible tickmark locations
    % % set(gca,'YTickLabel',{'\zeta_{min}', '-3 m', '-2 m', '-1 m', '0', '1 m', '2 m', '3 m', '\zeta_{max}'}) %informative labels at the tickmarks assuming -5 and +5 are positional limits
    % % set(gca, 'TickDir', 'out');
    % box off %Removes the borders of the plot area
    % xlabel('Time (s) →',  'FontWeight', 'bold') %Note use of unicode arrow for clarity
    % ylabel({'↑'; 'u'; ''},  'FontWeight', 'bold', 'Rotation',0, 'VerticalAlignment','middle', 'HorizontalAlignment','right') %Note cell matrices for line breaks - Tidy with right orientation (If all our vertical axes have the same internal offset all our axis labels will be neatly aligned
    % title(['Test: ' test_name], 'Interpreter', 'tex');
    % legend('fu_x', 'fu_y', 'fu_z', 'tau_u\phi', 'tau_u\theta', 'tau_u\psi', 'Location', 'best');    
    % if save_vis
    %     savefig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_Pos_',date_str,'.fig'));
    %     export_fig(strcat(save_loc, 'Fig_Comp_',exp_name,'_',test_name,'_Pos_',date_str,'.png'), '-m3', '-nocrop');
    % end

end

% Save updated results with metrics if needed
disp('Visualization completed.');
if save_vis
    disp(strcat('Figures saved - ',save_loc));
else
    disp('Figures not saved');
end