% Helper functions
function R = rotation_matrix(eta)
% Rotation matrix R(eta) from body to inertial (full Euler)
phi = eta(1); theta = eta(2); psi = eta(3);

% R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
% R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
% R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

R_phi = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
R_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R_psi = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];

R = R_phi * R_theta * R_psi; % or standard order
end
