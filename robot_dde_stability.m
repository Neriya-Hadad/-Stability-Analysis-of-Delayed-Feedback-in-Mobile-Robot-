% ================================
% Solving Delay Differential Equations (DDEs) with dde23
% Includes the computation of the unperturbed trajectory,
% conversion to a rotated coordinate frame, and computation
% of a delayed state-dependent variable delta_y_l(t - tau)
% ================================

% === System Parameters ===
a_l = 3;          % Coefficient in the equation for delta_x_l(t)
tau = 0.085;      % Delay value (in seconds)
a = 2;          % Parameter in the second-order equation for delta_alpha(t)
b = 1;            % Parameter in the second-order equation for delta_alpha(t)
v = 10;           % Constant forward velocity of the robot (m/s)
alpha = pi;       % Rotation angle of the reference frame (180 degrees)

% === Initial Robot Position ===
x0 = 1;           % Initial x-position in the world frame
y0 = 0;           % Initial y-position in the world frame
tspan = [0, 15];   % Simulation time interval in seconds
t_vec = linspace(tspan(1), tspan(2), 1000);  % Time vector for reference trajectory

% === Unperturbed (Nominal) Trajectory in World Frame ===
x_base = v * cos(alpha) * t_vec + x0;  % x(t) = v * cos(alpha) * t + x0
y_base = v * sin(alpha) * t_vec + y0;  % y(t) = v * sin(alpha) * t + y0

% === Conversion to Rotated (Local) Coordinate Frame ===
x_l_base = x_base * cos(alpha) + y_base * sin(alpha);
y_l_base = -x_base * sin(alpha) + y_base * cos(alpha);

% === History Function for DDE Solver ===
% Provides initial values for the delayed state variables:
% [delta_x_l(t); delta_alpha(t); d/dt delta_alpha(t)] for t <= 0
history = @(t) [0.1; 0.05; 0];  % Small initial perturbation

% === Define the Right-Hand Side (RHS) of the DDE system ===
% The RHS is implemented in a separate function 'ddefun_robot'
ddefun = @(t, y, Z) ddefun_robot(t, y, Z, a_l, a, b, v);

% === Solve the DDE System using MATLAB’s dde23 Solver ===
sol = dde23(ddefun, tau, history, tspan);

% === Plot the Components of the Solution ===
% sol.y(1,:) corresponds to delta_x_l(t)
% sol.y(2,:) corresponds to delta_alpha(t)
% sol.y(3,:) corresponds to d/dt delta_alpha(t)
figure;
plot(sol.x, sol.y(1,:), 'r', sol.x, sol.y(2,:), 'b', sol.x, sol.y(3,:), 'g');
legend('\delta x_l(t)', '\delta \alpha(t)', 'd/dt \delta \alpha(t)');
xlabel('Time t (s)');
ylabel('State Values');
title('Solution of DDE System using dde23');
grid on;

% === Compute delta_y_l(t - tau) Based on Analytical Formula ===
% According to the theoretical derivation:
% delta_y_l(t - tau) = -(2*b * delta_alpha(t - tau) + delta_alpha_dot(t)) / a
delta_y_l = zeros(size(sol.x));  % Preallocate result vector

for i = 1:length(sol.x)
    t_now = sol.x(i);
    t_past = t_now - tau;

    if t_past >= 0
        % Evaluate delayed states at (t - tau)
        delayed_vals = deval(sol, t_past);     % Returns [delta_x_l; delta_alpha; delta_alpha_dot]
        delta_alpha_tau = delayed_vals(2);     % Extract delta_alpha(t - tau)

        % Evaluate current state at time t
        current_vals = deval(sol, t_now);
        delta_alpha_dot = current_vals(3);     % Extract d/dt delta_alpha(t)

        % Compute delta_y_l(t - tau) using the given expression
        delta_y_l_t_minus_tau = -(2 * b * delta_alpha_tau + delta_alpha_dot) / a;

        % Store the result
        delta_y_l(i) = delta_y_l_t_minus_tau;
    end
end

% === Plot the Computed delta_y_l(t - tau) ===
figure;
plot(sol.x, delta_y_l, 'm', 'LineWidth', 1.5);
xlabel('Time t (s)');
ylabel('\delta y_l(t - \tau)');
title('Computed \delta y_l(t - \tau) from DDE Solution');
grid on;
