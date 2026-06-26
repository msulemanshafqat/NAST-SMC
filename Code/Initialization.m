% Initialization.m is the file to initialize all parameters for drone,
% simulation parameters for sim, and gain parameters for controllers.
% Multiple updates to initialization file has been carried out. Previous
% versions are available in TBD folder.

% Update: after mass, J and tau_eq changes, the gains needed retuning.

% During PSO training of all attitude techniques, the cost criteria has been fixed on 27 Oct 2025 as : swarm.cost(p) = norm_rmse_att * 0.3 + norm_rmse_pos * 0.4 + norm_energy * 0.3;
% Results noted after each gain value initialization, for record.


% Update v4: Switched to a smaller Octo (m=6.8kg), retuned. Old results of octorotor1: m=36Kg removed from updated version

% (see updated version v5)

% Clear workspace and set random seed for repeatability
clearvars
close all
% clc
rng(42); % For repeatable random waypoints, if required

%% Plant parameters structure
params = struct();

% Mulirotor UAV Body related parameters:
params.numrotors=8;
params.m = 6.8; % Kg
params.J = diag([0.34, 0.34, 0.65]); % inertia (kg m^2)
params.g = 9.81; % gravity (m/s^2)
params.e3 = [0; 0; 1]; % unit vector
params.ARMLEN = 0.39; % arm length (m)  (i.e. 0.508 prop + 0.5 center body (1m x 1m circular) + 0.2 clearance]

% r = 0.2;
r = 0.254; % radius of propeller in m 
% Rotor Thrust = C_T ρ π R^4 omega^2 = TF * omega^2 (where TF = C_T ρ π R^4)
CT = 0.0107;
params.TF = CT * 1.225 * pi * r^4; % TF = 0.0001713
params.omega2max = (7550*2*pi/60)^2; % 624,469 (rad/sec)^2 --> Corresponds to Single Rotor Max Thrust = TF * omega2max = 73N ~ 8Kg

% torque_drag = - c_D * rho * pi * R^2 omega |omega| R^3 = - DF * omega |omega|
CD = 0.044;
params.DF = CD * 1.225 * pi * r^5; % drag factor = 0.00017902
% params.DF = 1.1e-7; % drag factor

% Power Consumption Factor Parameter
params.rotorPF = 0.00105; %RPM to power_consumed conversion factor of single rotor. (PF= Power Factor)


if params.numrotors == 8
    params.M = [params.ARMLEN*params.TF/sqrt(2), 0, -params.ARMLEN*params.TF/sqrt(2), -params.ARMLEN*params.TF, -params.ARMLEN*params.TF/sqrt(2), 0, params.ARMLEN*params.TF/sqrt(2), params.ARMLEN*params.TF; 
            -params.ARMLEN*params.TF/sqrt(2), -params.ARMLEN*params.TF, -params.ARMLEN*params.TF/sqrt(2), 0, params.ARMLEN*params.TF/sqrt(2), params.ARMLEN*params.TF, params.ARMLEN*params.TF/sqrt(2), 0; 
            -params.DF, params.DF, -params.DF, params.DF, -params.DF, params.DF, -params.DF, params.DF]; % allocation matrix
elseif params.numrotors == 4
    params.M = [0, -params.ARMLEN*params.TF, 0, params.ARMLEN*params.TF; 
            -params.ARMLEN*params.TF, 0, params.ARMLEN*params.TF, 0; 
            -params.DF, params.DF, -params.DF, params.DF]; % allocation matrix
end
%% Simulation parameters structure
sim_params = struct();
sim_params.dt = 0.001;
sim_params.stoptime = 20;
sim_params.tspan = 0:sim_params.dt:sim_params.stoptime; % time vector (s)
sim_params.x0 = [zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)]; % initial state [zeta; v; eta; Omega
sim_params.u = [zeros(3,1); zeros(3,1); 0]; % [power_consumed; fu; tau_u]
sim_params.testnum=1;

noise_var = 0;
uncertainty = 0;
fault_pct = 0;
fault_rotors =[];
delta = 0;            % sec
sigma_f = zeros(3,1);
sigma_tau = zeros(3,1);

% Metrics are always displayed unless optimization is running, Metrics are saved in xlsx when save_metric flag is true
% Figures are displayed when vis flg is true, figures are saved in IEEE format when save_vis is true. 
opt         = false;
vis         = true;
save_vis    = false;
save_metrics= false;
save_loc    = 'D:\ASNSTSTSMC_2025\Results\';
sim_params.opt= opt;


pos_tech={'pos_PID', 'pos_ASNSTSTSMC', 'pos_nocontrol', 'FracFITSMC_pos'};
sim_params.use_pos=pos_tech{1};
sim_params.cascaded_control = true;

% Initialize results structure
results = struct();
alltests = {'spiral', 'circle', 'square', 'random', 'unit', 'SoilMapping', 'PrecisionSparying'};

% For tests to be run during simulations (unless otherwise defined in
% individual test):
sim_params.test = {'spiral', 'circle', 'square', 'random'};
    
%% Define controller gains structure
gains = struct();

% During PSO training of all attitude techniques, the cost criteria has been fixed on 27 Oct 2025 as : swarm.cost(p) = norm_rmse_att * 0.3 + norm_rmse_pos * 0.4 + norm_energy * 0.3;
% Results noted after each gain value initialization, for record.

% Update: after mass, J and tau_eq changes, the gains need retuning now.
% Update: Switched to a smaller Octo (m=6.8kg), re-tuning now. Old results of octorotor1: m=36Kg removed from updated version
% Update: Switched again to even smaller Octo (m=0.6Kg)


% Gains for PID Position Control (before PSO)
% results not very bad
kp_zeta = 4;
kd_zeta = 5;    
ki_zeta = 0.2;

% Gains for PID Position Control (after partial PSO)
% Iteration number14 completed, with current best cost =0.10464 For Gains:61.663268       20.85031              0           6000      309.17538      6.6184568
% kp_zeta = 61.663268;
% kd_zeta = 20.85031;    
% ki_zeta = 0;

kp_zeta = 126.92832;
kd_zeta = 44.307073;    
ki_zeta = 0.50737836;

% === ALTITUDE LOOP (position z) – best values for 6.8 kg octrotor ===
% kp_alt   = 80;    kd_alt   = 130;    ki_alt = 8;
% kp_alt   = 80;    kd_alt   = 20;    ki_alt = 3;

% PSO Tuning Completed for _PIDZeta_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=30, Total iterations40.
% The best cost =11.9927 For Gains:10      5.94288            0

% fn_z  = 1.6;        % 1.6 Hz closed-loop bandwidth → extremely aggressive but stable
% zeta_z = 0.80;
% [kp_alt, kd_alt] = pd_from_freq(params.m, fn_z, zeta_z);
% ki_alt = 6;    % small integral to kill steady-state error (wind/payload)
% → gives:
% kp_alt  ≈ 680
% kd_alt  ≈ 92.5


% %  PSO Gains (working so good 28-11-2025 with PSO):
kp_alt=3500;
kd_alt=227.5947;
ki_alt=14.851807;

% RESULTS
%  ub = [1500, 400, 20]
% Iteration number21 completed, with current best cost =9.0149 For Gains:1500      86.549635      13.556767
% ub = [3500, 400, 20]
% Iteration number40 completed, with current best cost =8.9611 For Gains:3500       227.5947      14.851807

% PSO Tuning Completed for _PIDAlt_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=30, Total iterations40.
% The best cost =8.9611 For Gains:3500       227.5947      14.851807

% PSO === PIDZetaAlt  
% ub = [180, 125, 5, 8500, 520, 10]
% lb = [2, 2, 0, 800, 200, 0]
% PSO Tuning Completed for _PIDZetaAlt_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=20, Total iterations20.
% The best cost =0.072211 For Gains:126.92832      44.307073     0.50737836      6418.2964      275.08865      6.3779944

%  PSO Gains (01 Dec 2025 with PSO):
% kp_alt=6418.2964;
% kd_alt=275.08865;
% ki_alt=6.3779944;

gains.kp_zeta = [kp_zeta; kp_zeta; kp_alt];
gains.kd_zeta = [kd_zeta; kd_zeta; kd_alt];
gains.ki_zeta = [ki_zeta; ki_zeta; ki_alt];


% Gains for AS-NST-STSMC non-cascaded cummulative Control
gbest = [267.90678      752.78724      938.41397      241.22949      293.03673              0     0.28754806      515.43672           1500      67.359791      1110.0833      1409.1881      581.86173      1092.9968           1500];
gains.alpha_cum =   [gbest(1);gbest(1);gbest(2);gbest(3)];
gains.beta_cum =    [gbest(4);gbest(4);gbest(5);gbest(6)];
gains.gamma_cum =   gbest(7);
gains.gamma1_cum =  gbest(8);
gains.gamma2_cum =  gbest(9);   
gains.k1_hat0_cum=  [gbest(10);gbest(10);gbest(11);gbest(12)];   
gains.k2_hat0_cum=  [gbest(13);gbest(13);gbest(14);gbest(15)];   

% Iteration number27 completed, with current best cost =200.8742 For Gains:1201.4695      1138.7111     0.33621927      14.617655      1.2067055      330.68084      20.335716

% Iteration number3 completed, with current best cost =68.0208 For Gains:267.90678      752.78724      938.41397      241.22949      293.03673              0     0.28754806      515.43672           1500      67.359791      1110.0833      1409.1881      581.86173      1092.9968           1500

% Iteration number85 completed, with current best cost =165.5309 For Gains:59.02113      34.74143      68.86335      862.3207      580.6686         0.001     0.5653838      17.08119      2.937783      207.6815      246.8061      335.7346      30.38713      31.37305      37.67816

% Iteration number100 completed, with current best cost =168.8718 For Gains:89.80305      59.87841      36.10574      249.4527      215.9599         1e-06     0.4844334      41.95961      2.614654      120.7845      65.87025       100.619      19.57302       19.0816      40.29985


% Gains for AS-NST-STSMC Position Control
%Old gains for heavy drone
% gains.alpha_pos = 4;   % velocity error gain
% gains.beta_pos = 1;         % For non-singular sliding surface
% gains.gamma_pos = 0.5;     % Gains adaptive factor for k1_hat_pos
% gains.gamma1_pos = 0.5;     % Gains adaptive factor for k1_hat_pos
% gains.gamma2_pos = 0.5;     % Gains adaptive factor for k2_hat_pos
% gains.k1_hat0_pos = 1;      % Initial value for adaptive gain k1_hat_pos
% gains.k2_hat0_pos = 2;     % Initial value for adaptive gain k2_hat_pos

%New gains for light drone - very stable good results in cascaded control
gains.alpha_pos = 10.39068;   % velocity error gain
gains.beta_pos = 712.2022;         % For non-singular sliding surface
gains.gamma_pos = 0.5770334;     % Gains adaptive factor for k1_hat_pos
gains.gamma1_pos = 26.1787;     % Gains adaptive factor for k1_hat_pos
gains.gamma2_pos = 2.049975;     % Gains adaptive factor for k2_hat_pos
gains.k1_hat0_pos = 247.7262;      % Initial value for adaptive gain k1_hat_pos
gains.k2_hat0_pos = 19.66532;     % Initial value for adaptive gain k2_hat_pos


% Iteration number80 completed, with current best cost =30.0233 For Gains:10.39068      712.2022     0.5770334       26.1787      2.049975      247.7262      19.66532
% PSO Tuning Completed for _pos_ASNSTSTSMC_ with inertia w=0.6,cognitive c1=1.7,social c2=1.2, for num of particles=30, Total iterations120.
% The best cost =30.0233 For Gains:10.39068      712.2022     0.5770334       26.1787      2.049975      247.7262      19.66532



% Gains for AS-NST-STSMC for Attitude Control   -  Results for 6.5Kg drone Nov 2025
gains.p = 3;            % Terminal Sliding Surface
gains.q = 2;            % Terminal Sliding Surface
gbest = [72, 80, 0.4456442, 6.520189, 24.0609, 479.0644, 28.58685]; % Best among all so far.
gains.alpha =   gbest(1);   % AS-NST-STSMC angular velocity error gain
gains.beta =    gbest(2);   % For non-singular sliding surface
gains.gamma =   gbest(3);   % CASF, chattering vs Speed of convergence
gains.gamma1 =  gbest(4);   % Gains adaptive factor for k1_hat (to be tuned using DRL)
gains.gamma2 =  gbest(5);   % Gains adaptive factor for k2_hat (to be tuned using DRL)
gains.k1_hat0=  gbest(6);   % Initial value for adaptive gain k1_hat
gains.k2_hat0=  gbest(7);   % Initial value for adaptive gain k2_hat

% Gains for BFSMC
gains.BFSMC = struct();
gbest = [15.29983, 612.1857, 0.6559066, 0.6432719, 84.45578]; %PSO 1st
% gbest = [224.7793       575.197     0.7498679     0.8152489       134.637]; %PSO 2nd
gains.BFSMC.alpha   = gbest(1);
gains.BFSMC.beta0   = gbest(2);
gains.BFSMC.gamma   = gbest(3);
gains.BFSMC.epsilon = gbest(4);
gains.BFSMC.z       = gbest(5); % this parameter needs to be retuned, wrongly tuned as T probably

% Results for 6.8KG drone Nov 2025
% PSO Tuning Completed for _BFSMC_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=50, Total iterations80.
% 1st run
% The best cost =23.999 For Gains:15.29983      612.1857     0.6559066     0.6432719      84.45578
% 2nd run
% Converged at: Iteration number64 completed, with current best cost =2.9349 For Gains:224.7793       575.197     0.7498679     0.8152489       134.637

% Gains for CABDISTSMC

% CABDISTSMC Gains (IGWO-inspired; tune via PSO with wider tests)
gains.CABDISTSMC = struct();
% % Per-axis: k_a1..12 (single-int), k_b1..12 (double-int), v1..12 (weights)
% gains.CABDISTSMC.ka = 395.2060;  % Single-int gains (roll,pitch,yaw pairs)
% gains.CABDISTSMC.kb = 335.9765;        % Double-int gains
% gains.CABDISTSMC.v1 = 19.7370;         % Surface weights
% gains.CABDISTSMC.v2 = 0.7054;         % Surface weights
% % Reaching: beta, gamma, z (per DOF)
% gains.CABDISTSMC.beta = 239.1765;   % [roll,pitch,yaw]
% gains.CABDISTSMC.gamma = 0.5330;
% gains.CABDISTSMC.zeta = 0.5803;
% gains.CABDISTSMC.w0 = 9.3309;    % Super-twist integral gain
% gains.CABDISTSMC.T = 0.9594;    % Saturation bound
gbest = [3150.3554      411.86795      6.9491163      18.346511      724.00816     0.44033229     0.39283027       11.34572      1.5091546];
gains.CABDISTSMC.ka =    gbest(1);
gains.CABDISTSMC.kb =    gbest(2);
gains.CABDISTSMC.v1 =    gbest(3);
gains.CABDISTSMC.v2 =    gbest(4);
gains.CABDISTSMC.beta  = gbest(5);
gains.CABDISTSMC.gamma = gbest(6);
gains.CABDISTSMC.zeta =  gbest(7);        
gains.CABDISTSMC.w0 =    gbest(8);        
gains.CABDISTSMC.T =     gbest(9);



% PSO lb/ub (expand to 30+ dims; match paper bounds)
% In Tune_Gains_PSO_v3.m: lb = [0.1*ones(1,36)]; ub = [50*ones(1,24), 10*ones(1,18), 1*ones(1,18), 10, 1]; % ka,kb,v,beta,gamma,zeta,w0,T

% RESULTS of PSO
% 1st - pos_nocontrol
% Iteration number53 completed, with current best cost =1.8742 For Gains:395.206      335.9765      19.73697     0.7053829      239.1765     0.5330144     0.5802846      9.330907     0.9593716
% Iteration number54 completed, with current best cost =1.8742 For Gains:395.206      335.9765      19.73697     0.7053829      239.1765     0.5330144     0.5802846      9.330907     0.9593716
% Iteration number55 completed, with current best cost =1.8742 For Gains:395.206      335.9765      19.73697     0.7053829      239.1765     0.5330144     0.5802846      9.330907     0.9593716
% Iteration number56 completed, with current best cost =1.8742 For Gains:395.206      335.9765      19.73697     0.7053829      239.1765     0.5330144     0.5802846      9.330907     0.9593716
% gbest = 395.2060  335.9765   19.7370    0.7054  239.1765    0.5330    0.5803    9.3309    0.9594

% 2nd - PID_pos - Spiral test

% old v4 values (discontinued as baseline unstable)
% gains.CABDISTSMC_alpha=81.22871;
% gains.CABDISTSMC_beta0=228.7098;
% gains.CABDISTSMC_gamma=0.6540927;
% gains.CABDISTSMC_epsilon=0.6762915;
% gains.CABDISTSMC_T=161.4165;
% gains.CABDISTSMC_w0=0.01;
% gains.CABDISTSMC_z=8.50416;

% Results for new Oct Nov 2025
% PSO Tuning Completed for _CABDISTSMC_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=50, Total iterations80.
% The best cost =24.6261 For Gains:81.22871      228.7098     0.6540927     0.6762915      161.4165          0.01     0.7117786

% CABDISTSMC PSO Tuning Completed with inertia w=0.7,cognitive c1=1.8,social c2=1.2, for num of particles=40, iterations=30
% sim_params.test = {'spiral', 'circle'};
% The best cost =0.092411 For Gains:15.4794      9.88546    0.0511791          0.8     0.721289      8.50416          0.1
% Define gains to tune (7 dims : alpha beta0 gamma epsilon T w0 zb)

% Results for 0.3Kg Drone Jan 2026. Tuning for all exp.
% Iteration number70 completed, with current best cost =85.1392 For Gains:3150.3554      411.86795      6.9491163      18.346511      724.00816     0.44033229     0.39283027       11.34572      1.5091546




% Gains for FracFITSMC_att
gains.FracFITSMC_att=struct();
gains.FracFITSMC_att.alpha=1;
gains.FracFITSMC_att.beta=6.419;
gains.FracFITSMC_att.k1=0.0001;
gains.FracFITSMC_att.k2=1.8106;

% PSO for new 0.3 KG drone - Tuning for all exp (Ph-1 coarse)
% Iteration number12 completed, with current best cost =45.7818 For Gains:435.33303      935.19262      1212.4141      575.37997
% PSO for new 0.3 KG drone - Tuning for all exp (Ph-2 fine)
% Converged at: Iteration number84 completed, with current best cost =44.8116 For Gains:227.28282      905.85156       1188.406      813.03797

% Iteration number40 completed, with current best cost =28.6532 For Gains:698.25215      1284.6493      1270.4733      452.40379

% Iteration number37 completed, with current best cost =26.7733 For Gains:347.32148      888.22303      1356.1335      234.84864
% Iteration number100 completed, with current best cost =26.7733 For Gains:347.32148      888.22303      1356.1335      234.84864
% PSO Tuning Completed for _FracFITSMC_ with inertia w=0.6,cognitive c1=1.7,social c2=1.2, for num of particles=30, Total iterations100.
% The best cost =26.7733 For Gains:347.32148      888.22303      1356.1335      234.84864

% Gains for Frac_FITSMC_pos
gains.FracFITSMC_pos=struct();
gains.FracFITSMC_pos.alpha=0.8;
gains.FracFITSMC_pos.beta=5;
gains.FracFITSMC_pos.k1=0.1;
gains.FracFITSMC_pos.k2=1.8;

% Gains for NSTSTSMC
gains.NSTSTSMC.lambda=2;
gains.NSTSTSMC.alpha1=1.5;
gains.NSTSTSMC.alpha2=1;

% Gains for STSMC
gains.STSMC_alpha=40.158649;
gains.STSMC_ksw=1022.9225;
gains.STSMC_ki=803.30074;

% Results for new Octo Nov 2025
% PSO Tuning Completed for _STSMC_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=50, Total iterations80.
% The best cost =9.7138 For Gains:40.158649      1022.9225      803.30074

% Gains of DISMC
gains.DISMC_alpha=24.30547;
gains.DISMC_ksw=653.2541;
gains.DISMC_ki1=253.1675;
gains.DISMC_ki2=165.8874;

% Results for new Octo Nov 2025 (more iterations may improve results)
% PSO Tuning Completed for _DISMC_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=50, Total iterations80.
% The best cost =10.1263 For Gains:24.30547      653.2541      253.1675      165.8874

% Gains for ISMC
gains.ISMC_alpha=24.38232;
gains.ISMC_ksw=653.7954;
gains.ISMC_ki=325.4067;

% Results for new Oct Nov 2025
% PSO Tuning Completed for _ISMC_ with inertia w=0.7,cognitive c1=1.7,social c2=1.2, for num of particles=50, Total iterations80.
% The best cost =10.1262 For Gains:24.38232      653.7954      325.4067      61.03164

% for ISMC, DISMC, STSMC epsilon is Fixed, not tuned
gains.epsilon=0.3;

% Gains for PD Attitude Control (for testing Only)
% gains.kp_eta = 20;    % PD Attitude Control (PSO Optimized)
% gains.kd_eta = 30;    % 1.5297;    % PD Attitude Control (PSO Optimized)


% === ATTITUDE LOOP (roll/pitch) – use inertia, not mass! ===
fn_att   = 2.5;     % ~2.5 Hz bandwidth → very fast
zeta_att = 0.75;

% Ix ≈ Iy ≈ 0.34 kg·m²

[kp_eta, kd_eta, ~]   = pd_from_freq(params.J(1,1), fn_att, zeta_att);
% → kp ≈ 87, kd ≈ 8.16

% === YAW LOOP ===
fn_yaw   = 1.5;
zeta_yaw = 0.70;
[kp_yaw, kd_yaw] = pd_from_freq(params.J(3,3), fn_yaw, zeta_yaw);
% → kp_yaw ≈ 79, kd_yaw ≈ 9.8
gains.kp_eta=[kp_eta;kp_eta;kp_yaw];
gains.kd_eta=[kd_eta;kd_eta;kd_yaw];


% reference points for SMC gains for PD-informed bounds (discontinued)
% gains_anchor_rollpitch = init_smc_from_pd(params, 2.0, 'att_rp');  % 2 Hz roll/pitch
% gains_anchor_yaw = init_smc_from_pd(params, 1.5, 'att_yaw');  % 2 Hz roll/pitch

% disp(gains)
%% MPC parameters structure for Position Control (Discontinued)
% mpc_params = struct();
% mpc_params.N = 10;
% mpc_params.Q_zeta = eye(3);
% mpc_params.Q_v = 0.1 * eye(3);
% mpc_params.R_mat = 0.01 * eye(3);
% mpc_params.dt = 0.05;
% mpc_params.sigma_f_bound = 0.1; % disturbance bound for tube MPC
% mpc_params.v_min = [-5; -5; -inf]*20;
% mpc_params.v_max = [5; 5; inf]*20;
% mpc_params.u_f_min = [-inf; -inf; -inf]; %placeholder for later use
% mpc_params.u_f_max = [inf; inf; inf]; %placeholder for later use

%% Desired Trajectory for different Tests 
% Setting desired position, velocity and yaw vectors for each test
% Define functions per test
zeta_d_func = cell(length(alltests), 1);
v_d_func = cell(length(alltests), 1);
yaw_d_func = cell(length(alltests), 1);

% Test 1: Spiral trajectory - Ramp Discontinued as position control is working good
scale = 0.2;  % Softer trajectories
zeta_d_func{1} = @(t) scale * [t/2 * cos(t);            t/2*sin(t)               ; t/4]; % desired position
v_d_func{1}    = @(t) scale * [0.5*cos(t) - t/2*sin(t); 0.5*sin(t) + t/2*cos(t)  ; 1/4]; % desired velocity
yaw_d_func{1}  = @(t) 0; % spiral: constant 0 rad for stable heading

% Test 2: Circle trajectory
ramp_time=pi; 
scale_yaw = 0.2;   % Scale factor to reduce yaw aggressiveness
zeta_d_func{2} = @(t)       [ cos(t/2).*(t>ramp_time) + cos(ramp_time/2).*(t<=ramp_time); sin(t/2); t/20 .* (t<=5)+0.25 .*(t>5)];
v_d_func{2}    = @(t) 0.5 * [-sin(t/2).*(t>ramp_time) + cos(ramp_time/2).*(t<=ramp_time); cos(t/2); 0.1 .* (t<=5)              ];  % 0.5 taken common to reduce computational load
% yaw_d_func{2} = @(t) scale_yaw * ( atan2(cos(t/2), -sin(t/2)) .*(t>ramp_time) + ...
                                    % atan2(cos(t/2), cos(ramp_time/2)) .*(t<=ramp_time) );  % Softened tangent yaw (tangent to v)
yaw_d_func{2} = @(t) scale_yaw * sin(t/4);

% Test 3: Square trajectory (piecewise waypoints)
waypoints = [0 0 0; 0 0 1; 2 0 1; 2 2 1; 0 2 1; 0 0 1];
t_way = linspace(0,sim_params.stoptime,size(waypoints,1));
zeta_d_func{3} = @(t) interp1(t_way, waypoints, mod(t,sim_params.stoptime), 'linear', 'extrap')';
v_segments = diff(waypoints) ./ diff(t_way');
v_d_func{3} = @(t) interp1(t_way(1:end-1), v_segments, mod(t,sim_params.stoptime), 'previous', 'extrap')';
yaw_d_func{3}  = @(t) 0; % square: constant 0 rad (simple waypoint following)


% Test 4: Random waypoint navigation
z=1;
scale=2;
random_waypoints = [0 0 0 ; 0 0 z+rand ; scale*rand scale*rand  z+rand; scale*rand  scale*rand  z+rand; scale*rand  scale*rand  z+rand];
random_waypoints = [random_waypoints; random_waypoints(end,:)]; % appending last waypoint to ensure consitent trajectory at end of simulation
t_way_rand = linspace(0,sim_params.stoptime,size(random_waypoints ,1));
zeta_d_func{4} = @(t) interp1(t_way_rand, random_waypoints, mod(t,sim_params.stoptime), 'linear', 'extrap')';
v_segments_rand = diff(random_waypoints) ./ diff(t_way_rand');
v_d_func{4} = @(t) interp1(t_way_rand(1:end-1), v_segments_rand, mod(t,sim_params.stoptime), 'previous', 'extrap')';
% yaw_d_func{4} = @(t) interp1(t_way_rand(1:end-1), atan2(diff(random_waypoints(:,2)), diff(random_waypoints(:,1))), mod(t,20), 'previous', 0); % yaw for direction to next waypoint
yaw_d_func{4} = @(t) scale_yaw * sin(t/4);

% Test 5: Unit Step Response (altitude step z=1 m at t=1s; constant xy=0, yaw=0)
step_time_x = 2; step_height = 3;  % Unit step params
step_time_z = 5;
step_time_yaw = 10;
zeta_d_func{5} = @(t) [(t >= step_time_x) * step_height; 0; (t >= step_time_z) * step_height];  % Step in x and z
v_d_func{5} = @(t) [0; 0; 0];  % v_d=0 post-step (steady hover)
yaw_d_func{5} = @(t) (t >= step_time_yaw) * pi/6;  % Step in yaw

% Test 6 : Agricultural mission trajectory for soil moisture mapping in a 100 m × 100 m field.
% UAV alternates between 15 m altitude (thermal imaging) and 25 m (NDVI multispectral) across 25 waypoints.
% Simulation duration for this test set to 600 seconds.
num_waypoints = 25;
field_size = 100;               % 100 m x 100 m field
altitudes = [15, 25];           % alternating altitudes (m)
sim_time_agri = 600;            % seconds for this test
t_way_agri = linspace(0, sim_time_agri, num_waypoints);

% Generate waypoints arranged in a lawnmower pattern covering the field
rows = ceil(sqrt(num_waypoints));
cols = ceil(num_waypoints / rows);
x_lin = linspace(0, field_size, cols);
y_lin = linspace(0, field_size, rows);
[Xg, Yg] = meshgrid(x_lin, y_lin);
wp_xy = [Xg(:), Yg(:)];
wp_xy = wp_xy(1:num_waypoints, :); % trim to desired number

% Alternate altitudes between altitudes(1) and altitudes(2)
wp_z = repmat(altitudes, 1, ceil(num_waypoints/2));
wp_z = wp_z(1:num_waypoints)';

waypoints_agri = [wp_xy, wp_z];

% Ensure circular repeatability by appending the first waypoint at end
waypoints_agri = [waypoints_agri; waypoints_agri(1,:)];

% Position trajectory (linear interpolation between waypoints)
zeta_d_func{6} = @(t) interp1(t_way_agri, waypoints_agri, mod(t, sim_time_agri), 'linear')';

% Velocity trajectory: piecewise constant segment velocities computed from waypoint intervals
v_segments_agri = diff(waypoints_agri) ./ diff([t_way_agri, sim_time_agri]); % last diff uses wrap-around but safe due to appended point
% Use previous segment value for between-waypoint times
v_d_func{6} = @(t) interp1(t_way_agri(1:end-1), v_segments_agri, mod(t, sim_time_agri), 'previous', 'extrap')';

% Yaw for agricultural mission: simple slow sinusoidal yaw to avoid abrupt changes
yaw_rate = 0.02;            % rad/s slow yaw rate
yaw_amplitude = pi/6;       % max yaw excursion
yaw_d_func{6} = @(t) yaw_amplitude * sin(yaw_rate * t);
