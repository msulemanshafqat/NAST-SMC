% To recompute eta_d post-ode45 (using same funcs/params)

function eta_d = compute_eta_d(t, x, zeta_d_func, v_d_func, yaw_d_func, params, gains, use_mpc, mpc_params)
eta_d = zeros(length(t), 3);
for i = 1:length(t)
    ti = t(i);
    xi = x(i, :);
    zeta_i = xi(1:3); v_i = xi(4:6); eta_i = xi(7:9);  % current state
    zeta_d_i = zeta_d_func(ti); 
    v_d_i = v_d_func(ti);
    yaw_d_i = yaw_d_func(ti);
    
    % Outer loop (PD example; swap with MPC call if use_mpc=true)
    if use_mpc
        [fu_i, eta_d_partial_i] = outer_loop_mpc(zeta_i, v_i, zeta_d_i, v_d_i, eta_i, params.m, params.J, params.g, params.e3, rotation_matrix(eta_i), mpc_params.N, mpc_params.Q_zeta, mpc_params.Q_v, mpc_params.R_mat, mpc_params.dt, mpc_params.sigma_f_bound, mpc_params.v_min, mpc_params.v_max, mpc_params.fu_min, mpc_params.fu_max);
        eta_d(i, :) = [eta_d_partial_i(1:2)'; yaw_d_i]';
    else
        R_i = rotation_matrix(eta_i);
        e_zeta_i = zeta_d_i - zeta_i;
        de_zeta_i = v_d_i - v_i;
        hat_sigma_f = zeros(3,1);  % estimate
        fu_i = R_i' * (params.m * params.g * params.e3 + params.m * (gains.kp_zeta * e_zeta_i + gains.kd_zeta * de_zeta_i) - hat_sigma_f);
    end
    % Clamp fu for stability (from prior fix)
    fu_i = max(min(fu_i, [inf; inf; params.m * params.g * 5]), [0; 0; params.m * params.g * 0.1]);  % Clamp: thrust 0.1mg to 5mg, positive
    norm_fu = norm(fu_i);
    if norm_fu < 1e-3
        fu_i = params.m * params.g * params.e3;
        norm_fu = norm(fu_i);
    end
    dir_fu = fu_i / norm_fu;
    phi_d_i = asin(max(min(dir_fu(2), 1), -1));  % roll, clamped
    theta_d_i = atan2(dir_fu(1), dir_fu(3));  % pitch
    eta_d(i, :) = [phi_d_i; theta_d_i; yaw_d_i]';  % full eta_d

    
    % To test only attitude control with desired fixed attitude (unit test)
    % eta_d(i,:)=[pi/18;-pi/16;0];

end
end