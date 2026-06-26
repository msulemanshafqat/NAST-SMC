% Octrotor dynamics function using structures.
% x = [zeta(3); v(3); eta(3); Omega(3)] (12x1)
% x_d = [zeta_d(3); v_d(3); eta_d(3); Omega_d(3)] (12x1)

% Updates:
% V2 onwards is now using fixed time loop based computation, which is practical for Multirotor Applications.
% v4 is adusted for switch based control of all control techniques
% V5 updates CAPDISTSMC control tech as per Mughees Et Al Paper

function [x_array,x_d_array,u_array, diverged] = octrotor_dynamics(zeta_d_func, v_d_func, yaw_d_func, params, gains, sim_params, delta, fault_pct, fault_rotors, sigma_f, sigma_tau, noise_var)

% Initializing variables for stateful computation (integral, delay buffer, adaptive gains
integral_delta = zeros(3,1); 
integral_delta_pos = zeros(3,1);
integral_delta_cum = zeros(4,1);
integral_e_zeta = zeros(3,1);
integral_e_eta = zeros(3,1);
% integral_e_cum = zeros(4,1);
integral_s = zeros(3,1);

interval = round(delta / sim_params.dt);
delay_buffer = zeros(4,interval);
smoothness_filter = zeros(4,2*interval);
% delay_buffer = []; 

k1_hat = gains.k1_hat0 * ones(3,1); 
k2_hat = gains.k2_hat0 * ones(3,1); 
k1_hat_cum = gains.k1_hat0_cum .* ones(4,1); 
k2_hat_cum = gains.k2_hat0_cum .* ones(4,1); 
k1_hat_pos = gains.k1_hat0_pos * ones(3,1); 
k2_hat_pos = gains.k2_hat0_pos * ones(3,1); 

% Python related parameters
episode_reward = 0;
step_count = 0;
total_steps = 0;
event_triggered = false;
event_threshold = 0.5;
event_count = 0;
mass_est = params.m;
mass_est_prev = params.m;
alt_prev=0;
L=0.05;
python_update_interval = 50; % after every 50 simulation steps
MAX_STEPS = sim_params.stoptime / sim_params.dt / python_update_interval;
done = false;


% CABDISTSMC_beta_hat = gains.CABDISTSMC_beta0 * ones(3,1);
e_int1_att = zeros(3,1);  % Single-int: e_phi11, e_theta11, e_yaw11 (for x,y,z pos skipping as using cascaded control)
e_dint_att = zeros(3,1);  % Double-int: e_phi111, etc.
v_i = zeros(3,1);         % Super-twist integrals (per DOF)

BFSMC.beta_hat = gains.BFSMC.beta0 * ones(3,1);
integral_sat=zeros(3,1);
doubleintegral_sat=zeros(3,1);
vi=zeros(3,1);
deta_d = zeros(3,1);
ddeta_d = zeros(3,1);
eta_d_prev=zeros(3,1);
yaw_d_prev=0;
deta_d_prev=zeros(3,1);
u_tau_delayed=zeros(3,1);
tau_u_actual=zeros(3,1);
u4 = zeros(4,1);
M_fault = params.M;
TF_fault = params.e3 * params.TF * ones(1,params.numrotors);

if sigma_f(1) == 0
    dist_f=false;
    sigma_f_t=sigma_f;
else 
    dist_f=true;
end
hat_sigma_f = zeros(3,1);  % estimated disturbance (0 for nominal, or from observer)

if sigma_tau(1) == 0
    dist_tau=false;
    sigma_tau_t=sigma_tau;
else 
    dist_tau=true;
end
% hat_sigma_tau = zeros(3,1);  % estimated disturbance (0 for nominal, or from observer)

% Initializing output vectors
x_array=zeros(sim_params.stoptime/sim_params.dt-2,12);
x_d_array=zeros(sim_params.stoptime/sim_params.dt-2,12);
u_array=zeros(sim_params.stoptime/sim_params.dt-2,9); %[power_consumed; u_tau_delayed (3,1); u_alt_delayed (1,1); f_u_z_actual (1,1); tau_u_actual (3,1)]

diverged = false;
arr=1;

if strcmp(sim_params.techname,'NASTSMC')
    % init_matlab_udp_interface
end
% initial state [zeta; v; eta; Omega]
x=sim_params.x0(1:12);

%% Main Control Loop
for t=0:sim_params.dt:sim_params.stoptime
    % Unpacking state
    zeta = x(1:3);
    v = x(4:6);
    eta = x(7:9);
    Omega = x(10:12);

    %% Reference Block
    % Desired states from test-specific functions
    zeta_d = zeta_d_func(t);
    v_d = v_d_func(t);
    yaw_d = yaw_d_func(t);
    
    % To avoid square and random last waypoint wrongly as zero at time = 20
    if zeta_d(1)==0 && t==20
        zeta_d = zeta_d_func(t-sim_params.dt);       
        v_d = v_d_func(t-sim_params.dt);
        yaw_d = yaw_d_func(t-sim_params.dt);
    end
    
    %% Sensor Block
    % Adding noise to measurements, if specified via experiment No 3
    if noise_var > 0
        eta_meas = eta + sqrt(noise_var) * randn(3,1);
        Omega_meas = Omega + sqrt(noise_var) * randn(3,1);
    else
        eta_meas = eta;
        Omega_meas = Omega;
    end
    
    %% Disturbance Block
    if dist_f
        sigma_f_t   = sigma_f(t);
        % if t>sim_params_dt
        %     hat_sigma_f = sigma_f(t-sim_params.dt);  % estimated
        %     disturbance (from observer) need to be corrected
        % end
    end
    if dist_tau
        sigma_tau_t = sigma_tau(t);
    end

    %% Fault Block (modifying M and TF for specified rotors) - In Exp 2
    % if t >= 5 && ~isempty(fault_rotors) && fault_pct > 0  % Fault activates at t=5s
    %     for r = fault_rotors(:)'
    %         if r <= size(M_fault,2)
    %             M_fault(:,r) = M_fault(:,r) * (1 - fault_pct);
    %             TF_fault(:,r)=TF_fault(:,r) * (1 - fault_pct);
    %         end
    %     end
    % end
    
    % --- Simple but realistic fault injection for simplified actuation ---
    % Symmetric octrotor → losing n rotors reduces effectiveness proportionally
    % (very good approximation as rotors are evenly distributed)
    effectiveness = 1 - length(fault_rotors) / 8 * fault_pct;
    
    %% Outer-loop control: PD or MPC os AS-NST-STSMC
    R_eta = rotation_matrix(eta_meas);
    e_zeta = zeta_d - zeta;
    de_zeta = v_d - v; % approximate
    integral_e_zeta = integral_e_zeta + e_zeta * sim_params.dt;
    
    % Feedforward for desired velocity
    % currently zero for testing without feedforward
    ff_term = 0 * params.m * de_zeta;   % Weight 0.5*ff_term for balance.
    if sim_params.cascaded_control
        switch sim_params.use_pos
            case 'pos_ASNSTSTSMC'
                % AS-NST-STSMC for outer loop Position control (results better than PD Position Control)
                % taking absolute to avoid singular power for sliding surface, while preserving sign of de_zeta
                de_zeta_power = sign(de_zeta) .* ( abs(de_zeta) .^ (gains.p / gains.q) ); % real nonsingular power
                s_pos = e_zeta + (1 / gains.beta_pos) * de_zeta_power;    % nonsingular sliding surface
                delta_s_pos = s_pos ./ (abs(s_pos) + gains.gamma_pos);          % Sigmoid based CASF
                integral_delta_pos = integral_delta_pos + delta_s_pos * sim_params.dt; % persistent update of CASF integral (Euler approximation)
                u_f = R_eta' * (gains.alpha_pos * de_zeta + zeros(3,1) + k1_hat_pos .* abs(s_pos).^(1/2) .* delta_s_pos + k2_hat_pos .* integral_delta_pos); % d2_zeta_d placeholder as 0
                % Update adaptive gains (vectorized) (adaptive gains may be via DRL estimation
                k1_hat_pos = k1_hat_pos + gains.gamma1_pos * abs(s_pos).^(3/2)  * sim_params.dt;
                k2_hat_pos = k2_hat_pos + gains.gamma2_pos * abs(s_pos)         * sim_params.dt;
            case 'pos_PID'
                % Outer-loop PID for u_f
                u_f = R_eta' * (params.m * params.g * params.e3 + params.m * (gains.kp_zeta .* e_zeta + gains.kd_zeta .* de_zeta + gains.ki_zeta .* integral_e_zeta) + ff_term - hat_sigma_f);
            case 'pos_nocontrol'
                % u_f = zeros(3,1);
                u_f = R_eta' * (params.m * params.g * params.e3);
            % ===== NEW CASES FOR 2025 BASELINES =====
            case 'FracFITSMC_pos'  % Fractional FITSMC (Luo 2025)
                % Fractional approx. via Grünwald-Letnikov (simple Euler proxy)
                if ~exist("FracFITSMC_de_zeta_prev","var"), FracFITSMC_de_zeta_prev = de_zeta; end
                frac_de_zeta = de_zeta + gains.FracFITSMC_pos.alpha * (de_zeta - FracFITSMC_de_zeta_prev) / sim_params.dt;  % D^alpha de
                FracFITSMC_de_zeta_prev = de_zeta;
                s_pos = e_zeta + (1/gains.FracFITSMC_pos.beta) * abs(frac_de_zeta).^(gains.FracFITSMC_pos.alpha) .* sign(frac_de_zeta);
                % ST reaching w/ ESO proxy (wind/fault comp.)
                eso_hat_pos = 0.001 * sin(t);  % Simple ESO for disturb. (tune via PSO)
                ddzeta_d = zeros(3,1); % for simplicity
                u_f = params.m * (ddzeta_d + gains.kp_zeta .* e_zeta + gains.kd_zeta .* de_zeta - eso_hat_pos) ...
                         - gains.FracFITSMC_pos.k1 .* abs(s_pos).^0.5 .* sign(s_pos) - gains.FracFITSMC_pos.k2 .* s_pos;
        end
        % Clamping virtual control u_f to 10 mg
        % u_f = sign(u_f) .* max(min(abs(u_f), params.m * params.g * 10 * [1; 1; 1]), params.m * params.g * 10 * [-1; -1; -1]); 
        u_alt=u_f(3);
        norm_u_f = norm(u_f);
        if norm_u_f < 1e-5  % Safeguard small norm
            u_f = R_eta' * (params.m * params.g * params.e3 + 0.001 * params.m * e_zeta);  % Hover Thrust with small position bias
            norm_u_f = norm(u_f);
        end
        dir_u_f = real(u_f / norm_u_f);
        %% Desired attitudes from u_f (inverse kinematics for roll/pitch)
        phi_d = asin(max(min(dir_u_f(2), 1), -1));  % roll; Clamping asin arg to [-1,1]
        theta_d = atan2( -dir_u_f(1), max(sqrt(dir_u_f(2)^2 + dir_u_f(3)^2),1e-6) ); % pitch; Avoiding zero denominator
        
        eta_d_partial = [phi_d; theta_d; yaw_d];  % yaw from test-specific desired
        
        if strcmp(sim_params.use_pos, 'pos_nocontrol')
            % To test only attitude control with desired fixed attitude (att unit test)
            eta_d=[pi/18;-pi/16;pi/-24];
        else
            % eta_d = eta_d_partial;
            eta_d = max(min(eta_d_partial, [pi/3;pi/3;pi/2]), -[pi/3;pi/3;pi/2]);  % Clamp to ±60°
        end
        eta_d=asin(sin(eta_d)); % Correcting desired eta for angle inversion
        
        % yaw unwrapping
        % yaw_delta = eta_d(3) - yaw_d_prev;
        % eta_d(3) = eta_d(3) - sign(yaw_delta) * 2*pi * (abs(yaw_delta) > pi/2);
        
        % Desired yaw rate (numerical; analytical if yaw_d_func provides it)
        % dyaw_d = (yaw_d - yaw_d_prev) / sim_params.dt;
        % yaw_d_prev = eta_d(3);
                
        % Desired attitude differentials: (note: issues of noise amplification leading to instability
        % deta_d=asin(sin( (eta_d - eta_d_prev)/sim_params.dt )); % derivative of eta_d 
        % eta_d_prev=eta_d;  % updating prev eta_d value.
         
        % ddeta_d=asin(sin( (deta_d - deta_d_prev)/sim_params.dt )); % derivative of deta_d 
        % deta_d_prev=deta_d;  % updating prev eta_d value.
        
        % Simplifying desired attitude differentials (for troubleshooting issues, if required)
        deta_d = zeros(3,1);
        ddeta_d = zeros(3,1);
        % Note: For aggressive maneuvers, filter eta_d and use finite diff. But for spiral/circle — safe and sufficient.
    
        %% Inner Loop 
        
        % Attitude error
        e_eta = eta_d - eta_meas;
        de_eta = deta_d - RT_matrix(eta_meas) * Omega_meas;     % omega_meas converted to deta_meas
        
        
        
        % Nominal equivalent control for SMC (inverse dynamics to cancel selected plant dynamics ) 
        switch sim_params.techname
            case ''   % Mention specific techniques here to include equivalent control, not needed for HOSMC tech.
                % Equivalent control (nominal inverse dynamics, ignoring hat_sigma_tau for benchmark)
                % u_tau_eq = tau_u_actual - Omega .* params.J * Omega + sigma_tau ...              % Feedforward 
                %     - cross(Omega_meas, params.J * Omega_meas); % Coriolis / Gyroscopic compensation (cancel Ω×JΩ)
                u_tau_eq = params.J * ddeta_d + cross(Omega_meas, params.J * Omega_meas); % Coriolis / Gyroscopic compensation
            otherwise
                % Simple control law, without equivalent term / compensation
                u_tau_eq = zeros(3,1); % Placeholder, zero for simplicity

                % Nominal equivalent control law, with ddeta_d compensation
                % u_tau_eq = ddeta_d; 
        end

        % Technique specific attitude control code
        switch sim_params.techname
            case 'nocontrol'
                u_tau=zeros(3,1);
            case 'PD'
                % PD Control
                u_tau = gains.kp_eta .* e_eta + gains.kd_eta .* de_eta;
            case 'NASTSMC'    % DRL to be used for adaptive gain tuning
                % Neural-AS-NST-STSMC (main control law)
                de_eta_abs = abs(de_eta);% taking absolute to avoid singular power for sliding surface, while preserving sign of de_eta
                de_eta_power = sign(de_eta) .* (de_eta_abs .^ (gains.p / gains.q)); % real nonsingular power
                s = e_eta + (1 / gains.beta) * de_eta_power;    % nonsingular sliding surface
                delta_s = s ./ (abs(s) + gains.gamma);          % Sigmoid based CASF
                integral_delta = integral_delta + delta_s * sim_params.dt; % persistent update of CASF integral (Euler approximation)
                
                % AS-NST-STSMC with nominal inverse dynamics
                % Full control; Robust term scaled by J for torque units
                % u_tau = u_tau_eq + params.J * (gains.alpha * de_eta + k1_hat .* abs(s).^(1/2) .* delta_s + k2_hat .* integral_delta);
            
                % Full control; Robust term without scaled by J (original)
                u_tau = u_tau_eq + gains.alpha * de_eta + k1_hat .* abs(s).^(1/2) .* delta_s + k2_hat .* integral_delta;                

                % Online adaptivity via Python dynamic PPO gain adjustments via
                if mod(arr, 50) == 0  % Every 50 iterations
                    matlab_udp_interface

                    % Compute reward
                    % reward = compute_reward(x, u4, []);
                    % episode_reward = episode_reward + reward;

                    % Send next state and reward to Python
                    % send_obs_to_python(udp_send, x, reward, done);
                    
                    % Receive Action from Python
                    % [gains.gamma1, gains.gamma2] = receive_action_from_python(udp_recv);
                end

                % Update adaptive gains (vectorized)
                
                dk1_hat = gains.gamma1 * abs(s).^(3/2);
                dk2_hat = gains.gamma2 * abs(s);
                
                k1_hat = k1_hat + dk1_hat * sim_params.dt;
                k2_hat = k2_hat + dk2_hat * sim_params.dt;
            
            case 'ETNASTSMC'    % Event Triggered Meta Learning with Python to be used for adaptive gain tuning
                % Event-Triggered-Neural-AS-NST-STSMC (2nd paper)
                de_eta_abs = abs(de_eta);% taking absolute to avoid singular power for sliding surface, while preserving sign of de_eta
                de_eta_power = sign(de_eta) .* (de_eta_abs .^ (gains.p / gains.q)); % real nonsingular power
                s = e_eta + (1 / gains.beta) * de_eta_power;    % nonsingular sliding surface
                delta_s = s ./ (abs(s) + gains.gamma);          % Sigmoid based CASF
                integral_delta = integral_delta + delta_s * sim_params.dt; % persistent update of CASF integral (Euler approximation)
                
                % AS-NST-STSMC with nominal inverse dynamics
                % Full control; Robust term scaled by J for torque units
                % u_tau = u_tau_eq + params.J * (gains.alpha * de_eta + k1_hat .* abs(s).^(1/2) .* delta_s + k2_hat .* integral_delta);

                if ~exist('u_st',"var")
                    % Initial Super-twisting control (update control
                    % signal) for later zero order hold
                    u_st = k1_hat .* abs(s).^(1/2) .* delta_s + k2_hat .* integral_delta;
                end

                % Event-trigger check
                delta_alt = alt_prev - zeta(3);
                alt_prev=zeta(3);
                
                % dmass_est = - L * (mass_est - mass_measured);
                dmass_est=0;
                mass_est = mass_est_prev + dmass_est * sim_params.dt;
                delta_mass_est = mass_est_prev - mass_est;
                mass_est_prev=mass_est;
                
                if norm(s) > event_threshold ...
                    || abs(delta_alt) > 0.5 ...          % delta pos in m
                    || abs(delta_mass_est) > 0.05 ... % delta m in kg
                    || norm(sigma_f_t) > 7                    % winds in m/s
                    event_triggered = true;
                    event_count = event_count + 1;
                    
                    % Super-twisting control (update control signal)
                    u_st = k1_hat .* abs(s).^(1/2) .* delta_s + k2_hat .* integral_delta;
                else
                    event_triggered = false;
                    % Zero-order hold (no control update)
                end
                
                % Full control; Robust term without scaled by J (original)
                u_tau = u_tau_eq + gains.alpha * de_eta + u_st;
                
                % Online adaptivity of gamma via Python
                if mod(arr, python_update_interval) == 0  % Every 50 iterations                    
                    ET_matlab_udp_interface
                    % uav_state = x;
                    control_effort = u4;

                    % Compute reward
                    reward = compute_reward(e_zeta, control_effort, event_triggered);
                    episode_reward = episode_reward + reward;

                    % Pack next 9D observation for DRL agent
                    % next_obs = [
                    %     s;              % [1:3] Sliding variable
                    %     v;              % [4:6] Velocity
                    %     zeta(3);        % [7] Altitude (need to change this to provide full position info)
                    %     params.m;       % [8] Mass
                    %     norm(sigma_f_t) % [9] Wind magnitude
                    % ];

                    % Check termination conditions
                    % if step_count >= MAX_STEPS || is_terminal_state(uav_state)
                    %     done = true;
                    % end
                    
                    % Send next obs and reward to Python
                    % send_obs_to_python(udp_send, next_obs, reward, done);
                    
                    % Update counters
                    step_count = step_count + 1;
                    total_steps = total_steps + 1;
                    
                    % Update current obs
                    % obs = next_obs;
                    
                    % Progress indicator (every 100 steps)
                    % if mod(step_count, 100) == 0
                    %     fprintf('  Step %d/%d | Reward: %.2f | Pos Error: %.3f m\n', ...
                    %         step_count, MAX_STEPS, episode_reward, norm(e_zeta));
                    % end
                    

                    % Receive action from Python agent
                    % action = receive_action_from_python(udp_recv);
                    action = [0,0];

                    if isempty(action)
                        fprintf('⚠ Timeout: No action received. Terminating episode.\n');
                        diverged = true;
                        break;
                    end

                    % Apply action: Update gamma delta for adaptive gains 
                    gamma1_delta = action(1);  % ∈ [-1, 1]
                    gamma2_delta = action(2);
                    
                    % gamma update (bounded)
                    gains.gamma1 = gains.gamma1 .* (1 + 0.15 * gamma1_delta); % ±15% adjustment
                    gains.gamma2 = gains.gamma2 .* (1 + 0.35 * gamma2_delta); % ±35% adjustment
        
                end

                % Update adaptive gains (vectorized)
                
                dk1_hat = gains.gamma1 * abs(s).^(3/2);
                dk2_hat = gains.gamma2 * abs(s);
                
                k1_hat = k1_hat + dk1_hat * sim_params.dt;
                k2_hat = k2_hat + dk2_hat * sim_params.dt;
            
            case 'ISMC'
                integral_e_eta = integral_e_eta + e_eta * sim_params.dt;
                s = de_eta + gains.ISMC_alpha * e_eta + gains.ISMC_ki * integral_e_eta;
                sat_s = s / gains.epsilon;
                sat_s = max(-1, min(1, sat_s));  % Equivalent to sat(s/epsilon)
                u_tau = u_tau_eq + gains.ISMC_alpha * de_eta + gains.ISMC_ksw * sat_s;

            % case 'ISMC'
            %     s = de_eta + e_eta * gains.ISMC_alpha;    % Standard sliding surface    
            %     sat_s = max(-gains.epsilon, min(gains.epsilon, s)); % Saturation function
            %     integral_sat = integral_sat + sat_s * sim_params.dt;
            %     u_tau = u_tau_eq + gains.ISMC_alpha * de_eta + gains.ISMC_ksw .* sat_s + gains.ISMC_ki .* integral_sat; 
                
            case 'DISMC'
                s = de_eta + e_eta * gains.DISMC_alpha;    % Standard sliding surface    
                sat_s = max(-gains.epsilon, min(gains.epsilon, s)); % Saturation function
                integral_sat = integral_sat + sat_s * sim_params.dt;
                doubleintegral_sat = doubleintegral_sat + integral_sat * sim_params.dt;
                u_tau = u_tau_eq + gains.DISMC_alpha * de_eta + gains.DISMC_ksw .* sat_s + gains.DISMC_ki1 .* integral_sat + gains.DISMC_ki2 .* doubleintegral_sat; 
            
            case 'STSMC'
                s = de_eta + e_eta .* gains.STSMC_alpha;    % Standard sliding surface    
                twisting_s = (abs(s)).^(1/2);         % Super Twisting term
                sat_s = max(-gains.epsilon, min(gains.epsilon, s)); % Saturation function
                integral_sat = integral_sat + sat_s * sim_params.dt;
                u_tau = u_tau_eq + gains.STSMC_alpha * de_eta + gains.STSMC_ksw .* twisting_s .* sat_s + gains.STSMC_ki .* integral_sat; 
            
            case 'BFSMC'
                s = de_eta + e_eta * gains.BFSMC.alpha;    % Standard sliding surface    
                % BFSMC.beta is gain of Adaptive barrier function 
                if abs(s)<gains.BFSMC.epsilon
                    BFSMC.beta_hat = abs(s) ./ (gains.BFSMC.epsilon - abs(s));
                else
                    BFSMC.beta_hat = BFSMC.beta_hat .* abs(s).^gains.BFSMC.gamma;   %gamma (0,1) is the adaptive factor for gain BFSMC.beta
                end
                u_tau = u_tau_eq + gains.BFSMC.alpha * de_eta + BFSMC.beta_hat .* (abs(s)).^gains.BFSMC.gamma .* sign(s/gains.BFSMC.z);
            case 'CABDISTSMC'
                % Update integrals (Euler, dt=sim_params.dt)
                e_int1_att = e_int1_att + e_eta * sim_params.dt;
                e_dint_att = e_dint_att + e_int1_att * sim_params.dt;  % Double-int
                % Sliding surfaces (Eq.54, vectorized; DOF idx 1:phi/roll,2:theta/pitch,3:psi/yaw,4:x,5:y,6:z)
                s_CAB = gains.CABDISTSMC.ka .* e_int1_att + ...  % ka * e11 (single-int; pos e_zeta as e1 proxy)
                        gains.CABDISTSMC.kb .* e_dint_att + ...  % kb * e111 (double-int)
                        gains.CABDISTSMC.v1 .* e_eta      + ...  % v1 * e (e)
                        gains.CABDISTSMC.v2 .* de_eta;           % v2 * dote (de)
                % Barrier conditioning (simple log barrier proxy; paper implicit in BF)
                barrier = log(1 + abs(s_CAB));  % Enforce |S| < constraint; tune
                s_CAB = s_CAB .* (1 ./ (1 + barrier));  % Conditioned s_CAB
                
                % Super-twist reaching (Eq.57-59, vectorized)
                U_base = -gains.CABDISTSMC.beta .* abs(s_CAB).^gains.CABDISTSMC.gamma .* sign(s_CAB ./ gains.CABDISTSMC.zeta);
                U_sat = U_base;  % Per-DOF sat
                idx_sat = abs(U_base) > gains.CABDISTSMC.T;
                U_sat(idx_sat) = gains.CABDISTSMC.T * sign(U_base(idx_sat));
                % Update v_i (Eq.58)
                dv_i = gains.CABDISTSMC.w0 * sign(v_i - U_sat);
                v_i = v_i + dv_i * sim_params.dt;
                % Full reaching
                % U_reach = U_base - v_i;
                
                % Control laws (Eq.63, e.g., for tau_u = [U2; U3; U4] att; adapt for pos)
                % Assume dynamics: tau = J * ( -cross(J Omega, Omega) + U_reach + ff_terms )
                % Full: e.g., U2 (roll) = 1/(v1 b1) * [ -(beta1 |Sc1|^g1 sign + v1) + dynamics comp ]
                % b = diag([params.J(1,1), params.J(2,2), params.J(3,3)]);  % b1=Jxx, etc. (paper b1=v1? adapt)
                u_tau =   gains.CABDISTSMC.beta .* abs(s_CAB).^gains.CABDISTSMC.gamma .* ...
                          sign(s_CAB ./ gains.CABDISTSMC.zeta) + v_i ;% + ...  % Reaching
                          % gains.CABDISTSMC.v .* zeros(3,1) - ...  % FF (dOmega_d = zeros)
                          % cross(params.J * Omega_meas, Omega_meas) - ...  % Coriolis
                          % 0 );  % Gyro/aero (neglected for octo - minimal impact) %[params.q1*eta(2)*Omega(3); params.q4*eta(1)*Omega(3); params.q7*eta(1)*eta(2)]
                
                % Chattering reduction: Smooth sign with approx_sign(s_CAB, epsilon) if needed
                % approx_sign = @(s, eps) (s ./ (abs(s) + eps));
                % Apply if high-freq: tau_u = tau_u .* approx_sign(s_CAB, gains.CABDISTSMC_epsilon);
            
            % === New Cases from 2023 - 2024 research =====
            
            % case 'BS_SMC'          % Backstepping Super-Twisting SMC (2023–2024 standard)
            %     % Virtual control (backstepping)
            %     z1 = zeta_meas - zeta_d;
            %     z2 = dzeta_meas - (-gains.kp_zeta*z1);   % alpha = -kp z1
            %     % Super-twisting on z2
            %     s = z2 + gains.kp_zeta*z1;
            %     ds = -gains.k1*abs(s)^0.5*sign(s) - gains.k2*s;
            %     integral_st = integral_st + gains.k2*sign(s)*dt;
            %     u_z = 1/params.m*(params.m*ddzeta_d - gains.kp_zeta*dzeta_meas ...
            %          - gains.kd_zeta*dzeta_meas + params.m*g) ...
            %          - gains.k1*abs(s)^0.5*sign(s) - integral_st;
            % 
            % case 'FST_SMC'         % Fuzzy Super-Twisting SMC (Ullah 2024, PLOS ONE)
            %     e_z = zeta_d(3) - zeta_meas(3);
            %     de_z = dzeta_d(3) - dzeta_meas(3);
            %     s = de_z + gains.lambda*fuzzy_gain( abs(e_z), gains );  % fuzzy_gain = Mamdani 5-rule
            %     % Simple triangular fuzzy (replace with your fuzzy engine if you have one)
            %     fuzzy_k = 30 + 70*tanh(5*abs(e_z));   % approximates fuzzy adaptation
            %     u_z = 1/params.m*(params.m*ddzeta_d + params.m*g) ...
            %          - fuzzy_k*( gains.alpha1*abs(s)^0.5*sign(s) + gains.alpha2*integral(sign(s)) );
            % 
            % case 'MRAC'            % Model Reference Adaptive Control (L'Afflitto 2024)
            %     % Reference model (critically damped)
            %     zeta_m = zeta_m + (dzeta_m)*dt;
            %     dzeta_m = dzeta_m + (-2*wn*dzeta_m - wn^2*(zeta_m - zeta_d(3)))*dt;
            %     wn = 8; persistent zeta_m dzeta_m; if isempty(zeta_m), zeta_m=zeta_meas(3); dzeta_m=0; end
            %     e_m = zeta_m - zeta_meas(3);
            %     % Adaptive law (Lyapunov)
            %     gamma_adapt = 120;
            %     theta_hat = theta_hat + gamma_adapt*e_m*(zeta_m)*dt;
            %     persistent theta_hat; if isempty(theta_hat), theta_hat = params.m; end
            %     u_z = theta_hat*(ddzeta_d + 2*wn*dzeta_m + wn^2*(zeta_d(3)-zeta_m)) + params.m*g;
            % 
            % case 'iMPC'            % Interpolating MPC (simple 200 ms horizon, 2024 trend)
            %     Np = 10;  % 10-step = 200 ms @ 50 Hz
            %     persistent u_prev; if isempty(u_prev), u_prev = params.m*g*ones(Np,1); end
            %     % Very light linearised prediction (A,B from hover)
            %     A = [1 dt; 0 1]; B = [0; dt/params.m];
            %     x_pred = zeros(2,Np+1); x_pred(:,1) = [zeta_meas(3)-zeta_d(3); dzeta_meas(3)];
            %     for k = 1:Np
            %         x_pred(:,k+1) = A*x_pred(:,k) + B*(u_prev(k)-params.m*g);
            %     end
            %     ref = zeta_d(3) + (1:Np)'*dzeta_d(3)*dt;
            %     H = 2*(B'*B); f = 2*B'*(x_pred(:,1:end-1)' - ref);
            %     u_opt = -inv(H)*f; u_opt = u_opt(1) + params.m*g;
            %     u_z = max(min(u_opt, params.m*g + 30), params.m*g - 20);  % saturation
            %     u_prev = [u_prev(2:end); u_z];
            
            % ===== NEW CASES FOR 2025 BASELINES =====
            case 'FracFITSMC'  % Fractional FITSMC (Luo 2025)
                % Fractional approx. via Grünwald-Letnikov (simple Euler proxy)
                if ~exist("FracFITSMC_de_eta_prev", "var"), FracFITSMC_de_eta_prev = de_eta; end
                frac_de_eta = de_eta + gains.FracFITSMC_att.alpha * (de_eta - FracFITSMC_de_eta_prev) / sim_params.dt;  % D^alpha de
                FracFITSMC_de_eta_prev = de_eta;
                s = e_eta + (1/gains.FracFITSMC_att.beta) * abs(frac_de_eta).^(gains.FracFITSMC_att.alpha) .* sign(frac_de_eta);
                % ST reaching w/ ESO proxy (wind/fault comp.)
                eso_hat_att = 0.1 * sin(t);  % Simple ESO for disturb. (tune via PSO)
                % u_tau = params.J * (ddeta_d + gains.kp_eta .* e_eta + gains.kd_eta .* de_eta - eso_hat_att) ...
                %          - gains.FracFITSMC_att.k1 .* abs(s).^0.5 .* sign(s) - gains.FracFITSMC_att.k2 .* s;
                u_tau = (ddeta_d + gains.kp_eta .* e_eta + gains.kd_eta .* de_eta - eso_hat_att) ...
                         - gains.FracFITSMC_att.k1 .* abs(s).^0.5 .* sign(s) - gains.FracFITSMC_att.k2 .* s;
            
            case 'NSTSTSMC'       % Mod. NSTSTSM (Sci. Rep. 2025) - gains.NSTSTSMC.lambda=2; gains.NSTSTSMC.alpha1=1.5; gains.NSTSTSMC.alpha2=1;
                % Non-sing. terminal ST surface
                s = de_eta + gains.NSTSTSMC.lambda * (abs(e_eta).^(gains.NSTSTSMC.alpha1-1) .* e_eta);
                integral_s = integral_s + sign(s) .* sim_params.dt;
                % ST for formation proxy (single-UAV: self-tracking)
                u_tau = params.m * ddeta_d - gains.NSTSTSMC.alpha1 * abs(s).^0.5 .* sign(s) ...
                         - gains.NSTSTSMC.alpha2 * integral_s;
                % Chattering filter: approx_sign = s_pos / (abs(s_pos) + 0.01);
            
            case 'Event_NTSMC'   % Event-Driven NTSMC (Pal 2025) - Gains: eta=0.1, rho=0.5 (event thresh.)
                % Non-sing. terminal surface
                s = e_zeta + (de_zeta / gains.rho) * abs(e_zeta)^(gains.rho-1);
                % Event-trigger: if norm(s) > eta, update (else hold u_prev)
                persistent u_prev; if isempty(u_prev), u_prev = params.m * g * params.e3; end
                if norm(s) > gains.eta
                    u_zeta = params.m * (ddzeta_d + gains.kp_zeta * s + gains.kd_zeta * de_zeta) ...
                             - gains.k1 * abs(s)^gains.rho * sign(s);
                    u_prev = u_zeta;
                else
                    u_zeta = u_prev;  % Aperiodic
                end
            
            case 'MLP_AdaptiveSMC'  % MLP Neural Adaptive SMC (Fatemi 2025) - Simple 2-layer MLP proxy (use MATLAB nn)
                % MLP approx. (train offline via PSO; here: simple tanh net for gains)
                net_input = [e_zeta; de_zeta; t];  % Features
                hidden = tanh(5 * net_input');  % 1-layer proxy (gains.W1=5 PSO-tuned)
                adapt_gain = 10 + 20 * tanh(gains.W2 * hidden);  % W2=PSO
                s = e_zeta + de_zeta / adapt_gain;
                u_zeta = params.m * ddzeta_d + adapt_gain * (gains.kp_zeta * e_zeta + gains.kd_zeta * de_zeta) ...
                         - adapt_gain * sign(s);  % Adaptive switching
            case 'no_control'
                u_tau = zeros(3,1);
                u_f = R_eta' * (params.m * params.g * params.e3);
        end
    else
        % if not using cascaded control loop, then a cummulative 4dim error is to be controlled using the control technique.
        switch sim_params.techname
            case 'ASNSTSTSMC_cum' 
                % AS-NST-STSMC for cummulative control from positions and yaw errors to u_alt and u_tau
                % gains must be 4 dimmenssional
                % e_cum [roll;pitch;yaw;alt]
                e_cum =  [ e_zeta(2);  e_zeta(1);yaw_d-eta_meas(3);  e_zeta(3)];
                de_cum = [de_zeta(2); de_zeta(1); - Omega_meas(3) ; de_zeta(3)];
                
                % taking absolute to avoid singular power for sliding surface, while preserving sign of de_zeta
                de_cum_power = sign(de_cum) .* ( abs(de_cum) .^ (gains.p / gains.q) ); % real nonsingular power
                s_cum = e_cum + (1 ./ gains.beta_cum) .* de_cum_power;    % nonsingular sliding surface
                delta_s_cum = s_cum ./ (abs(s_cum) + gains.gamma_cum);          % Sigmoid based CASF
                integral_delta_cum = integral_delta_cum + delta_s_cum * sim_params.dt; % persistent update of CASF integral (Euler approximation)
                u4 = [[R_eta';0 0 0]'; 0 0 0 1]' * (gains.alpha_cum .* de_cum + k1_hat_cum .* abs(s_cum).^(1/2) .* delta_s_cum + k2_hat_cum .* integral_delta_cum); % d2_zeta_d placeholder as 0
                % Update adaptive gains (vectorized) (adaptive gains may be via DRL estimation
                k1_hat_cum = k1_hat_cum + gains.gamma1_cum .* abs(s_cum).^(3/2)  * sim_params.dt;
                k2_hat_cum = k2_hat_cum + gains.gamma2_cum .* abs(s_cum)         * sim_params.dt;
                u4(2) = -u4(2);
                eta_d = [zeta_d(2);zeta_d(1);yaw_d];

        end
        u_alt = u4(4);
        u_tau = u4(1:3);
    end
    % damping u_tau
    % u_tau(3) = u_tau(3)/30;
    % u_tau(3) = gains.kp_eta * e_eta(3) + gains.kd_eta * de_eta(3);
    
    % clamping u_tau
    % u_tau = sign(u_tau) .* max(min(abs(u_tau), params.m * params.g * 5 * [1; 1; 1]), params.m * params.g * 5 * [-1; -1; -1]);  % Clamp: thrust 0.1-3 mg, positive
    
    %% Apply delay (simple buffer approximation) - In Exp 2 (for alt and torque both)
    u4 = [u_tau; u_alt];
    if delta > 0
        delay_buffer = [delay_buffer(:, 2:end), u4];
        smoothness_filter = [smoothness_filter(:,2:end),u4];
        
        u_tau_delayed = delay_buffer(1:3,1);  % oldest
        u_alt_delayed = delay_buffer(4,1);

        % Mean of the control commands data - smoothness filter  -
        % (note: filter has double the data as of delay_buffer, we are using only half data, and skipping data not received yet at actuator

        % u_tau_delayed = mean(smoothness_filter(1:3,1:interval),2);
        % u_alt_delayed = mean(smoothness_filter(4  ,1:interval),2);
        
        % Smoothness filter discontinued due to poor results
    else
        u_tau_delayed = u_tau;
        u_alt_delayed = u_alt;
    end
    
    %% Full allocation: Solve for omega^2 using augmented matrix for u_alt and u_tau
    % update: M_fault is not used for rpm calculations as controller will
    % never know the fault in real time (unless a provision is separately
    % made)
    
    % u_alt_delayed = u_f(3)/params.TF; % Vertical thrust for testing old logic
    
    % N is Augmented allocation matrix
    N = [params.TF * ones(1,params.numrotors); params.M];  % 4xnumrotors matrix
    % N = [TF_fault * ones(1,params.numrotors); M_fault];  % temp testing M_fault (old logic)
    target = [u_alt_delayed; u_tau_delayed];  % 4x1
    omega2 = pinv(N) * target;  % least-squares solution for over-actuated
    omega2 = min(max(omega2, 0),params.omega2max);  % ensure non-negative (speed in rad/sec)^2 and within limit
    
    %% Inverse Dynamics for Actual f_u and tau_u from rpm allocation and Fault injection logic
    % Update: fault in Exp 2 (M_fault and TF_fault) is added in actual f_u and tau_u and
    % dynamics update as the fault imapact will be visible via sensors.
    
    % Apply faults in Thrust and Moment matrices
    f_u_actual  = TF_fault * omega2;  % vertical only
    tau_u_actual = M_fault * omega2;
    
    % Apply reduced effectiveness globally (thrust + torque) on control (instead of rpms)
    f_u_actual   = effectiveness * u_alt_delayed * params.e3;
    tau_u_actual = effectiveness * u_tau_delayed;
    
    %% Dynamics update
    dzeta = v;
    dv = (1/params.m) * rotation_matrix(eta_meas) * f_u_actual - params.g * params.e3 + sigma_f_t / params.m;
    deta = RT_matrix(eta_meas) * Omega_meas;
    dOmega = params.J \ (tau_u_actual + cross(params.J * Omega_meas, Omega_meas) + sigma_tau_t);

    power_consumed = sum(omega2) * params.rotorPF;  % Power consumed by all rotors combined at current rpm; Clamped omega2>=0 already before    
    
    % dzeta = real(dzeta); dv = real(dv); deta = real(deta); dOmega = real(dOmega);  % strip imaginary
    if any(isnan([dzeta; dv; deta; dOmega]) | isinf([dzeta; dv; deta; dOmega]))
        dxdt = zeros(12,1);  % zero out if NaN/Inf
        % warning('NaN/Inf detected at t=%f, resetting dxdt', t);
    else
        dxdt = [dzeta; dv; deta; dOmega];
    end
    
    x = x + dxdt * sim_params.dt;
    
    % Anti-windup protection for Euler Angles
    x(7:9)=asin(sin(x(7:9)));

    % Angular Velocity Constraint to physical drone limitations.
    x(10:12)=min(max(x(10:12),[-40; -40; -40]), [40; 40; 40]);

    % Enabling State Clamping for position and velocities to protect
    % against unreal numbers
    x(1:6)=min(max(x(1:6),[-100; -100; -100; -40; -40; -40]), [100; 100; 100; 40; 40; 40]);

    % Compiling array for returning
    x_array(arr,:)=x';
    x_d_array(arr,:) = [zeta_d; v_d; eta_d; deta_d]';
    u_array(arr,:)=[power_consumed; u_tau_delayed;  u_alt_delayed; f_u_actual(3); tau_u_actual]';
    
    % --- DIVERGENCE & SATURATION GUARD DURING OPTIMIZATION (Except for no control sanity check)---
    if sim_params.opt && ~sim_params.sanity_check
        if strcmp(sim_params.use_pos,'pos_nocontrol')
            if ~exist("divergence","var"), divergence=0; end
            divergence = mean([divergence,sqrt(mean(e_eta.^2))]);    % divergence of e_eta when testing fixed attitude control
        else
            divergence = sqrt(mean(e_zeta.^2));  % RMS position error
        end

        if divergence > 10  % 10 m RMS = clear instability
            diverged = true;
            % fprintf('  [DIVERGED] test# %d; tech# %d | t=%.2f s | RMS pos err = %.2f m\n', sim_params.testnum, sim_params.use_att, t, pos_err_rms);
            % fprintf('Warning: Outlier particle detected at time %f for test number %d and tech num %d.\n', t, sim_params.testnum, sim_params.use_att);
            break;
        end
    end
    % if (sim_params.testnum==3 || sim_params.testnum==4) && t==20
    %     x_d_array(arr,:) = x_d_array(arr-1,:);       
    % end
    arr=arr+1;
end
end