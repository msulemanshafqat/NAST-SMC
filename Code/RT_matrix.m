% helper function
function RT = RT_matrix(eta)
% R_T(eta) for dot eta = RT Omega
phi = eta(1); theta = eta(2); psi = eta(3);



RT = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
      0, cos(phi), -sin(phi);
      0, sin(phi)/cos(theta), cos(phi)/cos(theta)];


% RT = [ 1 0 0;
%           0 cos(phi) sin(phi);
%           0 -sin(phi) cos(phi) ]; % This is a simplified example

end
