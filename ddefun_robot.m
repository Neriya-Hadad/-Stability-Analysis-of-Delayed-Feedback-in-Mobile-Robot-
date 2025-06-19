function dydt = ddefun_robot(t, y, Z, a_l, a, b, v)
% DDEFUN_ROBOT
% Computes the derivatives for the robot's delay differential equations.
%
% Inputs:
%   t  - Current time
%   y  - Current state vector: [delta_x_l(t); delta_alpha(t); delta_alpha_dot(t)]
%   Z  - Delayed state vector at time t - tau: [delta_x_l(t - tau); delta_alpha(t - tau); delta_alpha_dot(t - tau)]
%   a_l, a, b, v - System parameters
%
% Output:
%   dydt - Derivatives vector: [d/dt delta_x_l; d/dt delta_alpha; d²/dt² delta_alpha]

    % Extract delayed states
    delta_x_l_tau     = Z(1);   % delta_x_l(t - tau)
    delta_alpha_tau   = Z(2);   % delta_alpha(t - tau)
    delta_alpha_dot_tau = Z(3); % delta_alpha_dot(t - tau)

    % Extract current state
    delta_alpha_dot = y(3);     % delta_alpha_dot(t)

    % Derivatives according to equations (3.3) and (3.4)
    dx_l_dt        = -a_l * delta_x_l_tau;
    d_alpha_dt     = delta_alpha_dot;
    d2_alpha_dt2   = -2 * b * delta_alpha_dot_tau - a * v * delta_alpha_tau;

    % Return as column vector
    dydt = [dx_l_dt; d_alpha_dt; d2_alpha_dt2];
end
