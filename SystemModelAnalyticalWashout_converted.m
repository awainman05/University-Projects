%% SystemModelAnalyticalWashout_converted.m
% Standalone MATLAB equivalent of SystemModelAnalyticalWashout.slx
%
% The active model path is:
%
%   Ay -> Saturation (+/-6)
%      -> analytical_cue
%      -> discrete washout transfer function
%      -> Gain 0.05
%      -> discrete integrator
%      -> vy
%
%   Position -> self-centering velocity -> Gain 5 ----\
%   Position -> pushback velocity      -> Gain 10 -----+-> Sum
%   vy ------------------------------------------------/
%      -> Rate Limiter (+5/-5)
%      -> Saturation (+/-0.3)
%      -> v_platformcmd
%
% The AKD controller, motor model and motion-platform model are also
% reproduced below.

clear;
clc;

%% ------------------------------------------------------------------------
% Load input data
% -------------------------------------------------------------------------

Ts = 0.001;     % Model sample time [s]

load("C:\Users\alexw\Documents\MATLAB\IRP\MonzaLap.mat");

% Original lap time
t_data = Lap_Time.Value(:);

% Acceleration data
Ay_data = CG_Accel_Longitudinal.Value(:);
Ax_data = CG_Accel_Lateral.Value(:);
Az_data = CG_Accel_Vertical.Value(:);

% Make sure all signals have the same number of samples
n = min([numel(t_data), numel(Ay_data), numel(Ax_data), numel(Az_data)]);

t_data  = t_data(1:n);
Ay_data = Ay_data(1:n);
Ax_data = Ax_data(1:n);
Az_data = Az_data(1:n);

% Remove duplicate time stamps.
% This is necessary because interp1 requires unique sample points.
[t_data, unique_idx] = unique(t_data, 'stable');

Ay_data = Ay_data(unique_idx);
Ax_data = Ax_data(unique_idx);
Az_data = Az_data(unique_idx);

T_end = t_data(end);

% Discrete simulation time
t = (0:Ts:T_end).';

if t(end) < T_end
    t = [t; T_end];
end

Nsim = numel(t);

% Resample the measured signals onto the 1 ms simulation grid.
Ay = interp1(t_data, Ay_data, t, 'linear', 'extrap');
Ax = interp1(t_data, Ax_data, t, 'linear', 'extrap');
Az = interp1(t_data, Az_data, t, 'linear', 'extrap');

Ay = Ay(:);
Ax = Ax(:);
Az = Az(:);

%% ------------------------------------------------------------------------
% Allocate signals
% -------------------------------------------------------------------------

% Control algorithm
Ay_limited    = zeros(Nsim,1);
v_pre         = zeros(Nsim,1);
W_ay          = zeros(Nsim,1);
W_ay_scaled   = zeros(Nsim,1);
vy            = zeros(Nsim,1);
v_self        = zeros(Nsim,1);
v_push        = zeros(Nsim,1);
v_sum         = zeros(Nsim,1);
v_rate_limited = zeros(Nsim,1);
vx_limited    = zeros(Nsim,1);
v_platformcmd = zeros(Nsim,1);

% AKD controller
w_feedback = zeros(Nsim,1);
w_ref       = zeros(Nsim,1);
velocity_error = zeros(Nsim,1);
I_unsat     = zeros(Nsim,1);
I_cmd       = zeros(Nsim,1);
T_cmd       = zeros(Nsim,1);

% Motor / mechanism
w_motor     = zeros(Nsim,1);
theta_motor = zeros(Nsim,1);
theta_crank = zeros(Nsim,1);
omega_crank = zeros(Nsim,1);
x           = zeros(Nsim,1);
v           = zeros(Nsim,1);

% Internal states
transfer_x1 = 0;
transfer_y1 = 0;
washout_integrator = 0;
rate_limiter_y = 0;
controller_integ = 0;
w_delay = 0;

%% ------------------------------------------------------------------------
% Parameters from SystemModelAnalyticalWashout.slx
% -------------------------------------------------------------------------

% Analytical cue
Ka = 0.5;
Kn = 0.5;
Vmax = 10;

% Washout filter
b0 = 0.998744;
b1 = -0.998744;
a1 = -0.997488;

% Washout gain
K_washout = 0.05;

% Position feedback gains
K_self = 5;
K_push = 10;

% Final rate limiter
rise_limit = 5;
fall_limit = -5;

% Final saturation
vx_min = -0.3;
vx_max = 0.3;

% AKD controller
washout_to_radps = 500;
Kp_AKD = 0.363;
Ki_AKD = 24.972;
Kt = 0.58;
I_peak = 18.0;

% Motor
J_motor = 3.12e-4;
B_motor = 0.076;
w_max = 502.65;

% Harmonic drive
N_gear = 100;

% Linkage
Lc = 0.050;
Lr = 0.075;
H = 0.080;

% Seat + driver
m_load = 100;

% Motor position limits
theta_crank_min = deg2rad(-21.9632);
theta_crank_max = deg2rad(18.4462);

theta_motor_min = N_gear * theta_crank_min;
theta_motor_max = N_gear * theta_crank_max;

% Initial motor state
w_motor(1) = 0;
theta_motor(1) = N_gear * deg2rad(7.81);

%% ------------------------------------------------------------------------
% Main simulation loop
% -------------------------------------------------------------------------

for k = 1:Nsim

    %% ====================================================================
    % CONTROL ALGORITHM
    % =====================================================================

    % Saturation4: Ay limited to +/-6
    Ay_limited(k) = min(max(Ay(k), -6), 6);

    % MATLAB Function: analytical_cue
    %
    % v_pre = Ka * Ay * (1 + Kn*abs(Ay))
    % v_pre = Vmax * tanh(v_pre/Vmax)

    v_pre(k) = Ka * Ay_limited(k) * ...
               (1 + Kn * abs(Ay_limited(k)));

    v_pre(k) = Vmax * tanh(v_pre(k) / Vmax);

    % Discrete Transfer Fcn1
    %
    % Numerator   = [0.998744 -0.998744]
    % Denominator = [1 -0.997488]

    W_ay(k) = -a1 * transfer_y1 + ...
              b0 * v_pre(k) + ...
              b1 * transfer_x1;

    transfer_x1 = v_pre(k);
    transfer_y1 = W_ay(k);

    % Gain = 0.05
    W_ay_scaled(k) = K_washout * W_ay(k);

    % Discrete-Time Integrator2
    %
    % Initial condition = 0
    washout_integrator = washout_integrator + Ts * W_ay_scaled(k);
    vy(k) = washout_integrator;

    %% ====================================================================
    % PLATFORM POSITION
    % =====================================================================

    theta_crank(k) = theta_motor(k) / N_gear;
    omega_crank(k) = w_motor(k) / N_gear;

    [x(k), v(k)] = motion_platform(theta_crank(k), ...
                                   omega_crank(k));

    % Motion Platform subsystem saturation: +/-0.17 m
    x(k) = min(max(x(k), -0.17), 0.17);

    %% ====================================================================
    % SELF-CENTERING
    % =====================================================================

    v_self(k) = self_centering_velocity(x(k));

    %% ====================================================================
    % PUSHBACK
    % =====================================================================

    v_push(k) = pushback_velocity(x(k));

    %% ====================================================================
    % SUM
    % ====================================================================

    v_sum(k) = vy(k) + ...
               K_self * v_self(k) + ...
               K_push * v_push(k);

    %% ====================================================================
    % RATE LIMITER
    % ====================================================================

    if k == 1
        v_rate_limited(k) = rate_limiter_y;
    else

        delta = v_sum(k) - rate_limiter_y;

        % Simulink Rate Limiter uses slew rate per second.
        max_rise = rise_limit * Ts;
        max_fall = fall_limit * Ts;

        if delta > max_rise
            delta = max_rise;
        elseif delta < max_fall
            delta = max_fall;
        end

        v_rate_limited(k) = rate_limiter_y + delta;
    end

    rate_limiter_y = v_rate_limited(k);

    %% ====================================================================
    % FINAL SATURATION
    % =====================================================================

    vx_limited(k) = min(max(v_rate_limited(k), ...
                            vx_min), vx_max);

    v_platformcmd(k) = vx_limited(k);

    %% ====================================================================
    % AKD CONTROLLER
    % =====================================================================

    % Root model contains a unit delay on motor velocity.
    w_feedback(k) = w_delay;

    % Platform command -> motor velocity reference
    w_ref(k) = v_platformcmd(k) * washout_to_radps;

    % Velocity error
    velocity_error(k) = w_ref(k) - w_feedback(k);

    % PI controller
    I_unsat(k) = Kp_AKD * velocity_error(k) + ...
                 Ki_AKD * controller_integ;

    % Current saturation
    I_cmd(k) = min(max(I_unsat(k), -I_peak), I_peak);

    % Anti-windup
    if (abs(I_unsat(k)) < I_peak) || ...
       ((I_unsat(k) > I_peak) && (velocity_error(k) < 0)) || ...
       ((I_unsat(k) < -I_peak) && (velocity_error(k) > 0))

        controller_integ = controller_integ + ...
                           velocity_error(k) * Ts;
    end

    % Current -> torque
    T_cmd(k) = Kt * I_cmd(k);

    %% ====================================================================
    % MOTOR MODEL
    % =====================================================================

    theta_now = theta_motor(k);
    w_now = w_motor(k);

    theta_crank_now = theta_now / N_gear;

    term_squared = ...
        Lr^2 - (H - Lc*cos(theta_crank_now))^2;

    term_squared = max(term_squared, 1e-12);
    term = sqrt(term_squared);

    dx_dtheta = ...
        Lc*cos(theta_crank_now) + ...
        (Lc*sin(theta_crank_now) * ...
        (H - Lc*cos(theta_crank_now))) / term;

    % Reflected linkage inertia
    J_load = m_load * dx_dtheta^2 / N_gear^2;

    J_total = J_motor + J_load;

    % Motor dynamics
    dw = (T_cmd(k) - B*w_now) / J_total;

    w_new = w_now + Ts * dw;

    % Motor speed limit
    w_new = min(max(w_new, -w_max), w_max);

    % Motor position limits
    theta_new = theta_now;

    if theta_now >= theta_motor_max

        theta_new = theta_motor_max;

        if w_new > 0
            w_new = 0;
        end

    elseif theta_now <= theta_motor_min

        theta_new = theta_motor_min;

        if w_new < 0
            w_new = 0;
        end
    end

    % Update motor position
    theta_new = theta_new + w_new * Ts;

    % Store next state
    if k < Nsim
        w_motor(k+1) = w_new;
        theta_motor(k+1) = theta_new;
    end

    % Unit-delay feedback
    w_delay = w_new;

end

%% ------------------------------------------------------------------------
% Results
% -------------------------------------------------------------------------

% Figure 1 - Input and analytical cue
figure;
plot(t, Ay, 'DisplayName', 'Ay input');
hold on;
plot(t, Ay_limited, 'DisplayName', 'Ay after saturation');
plot(t, v_pre, 'DisplayName', 'Analytical cue output');
hold off;
grid on;
xlabel('Time [s]');
ylabel('Signal');
title('Analytical Cue');
legend('Location', 'best');


% Figure 2 - Washout filter
figure;
plot(t, W_ay, 'DisplayName', 'Washout filter output');
hold on;
plot(t, W_ay_scaled, 'DisplayName', 'Scaled washout output');
plot(t, vy, 'DisplayName', 'Integrated velocity v_y');
hold off;
grid on;
xlabel('Time [s]');
ylabel('Signal');
title('Washout Filter Response');
legend('Location', 'best');


% Figure 3 - Platform position
figure;
plot(t, x * 1000);
grid on;
xlabel('Time [s]');
ylabel('Platform Position [mm]');
title('Platform Position');


% Figure 4 - Final platform command
figure;
plot(t, v_platformcmd);
grid on;
xlabel('Time [s]');
ylabel('Platform Command');
title('Final Platform Velocity Command');


% Figure 5 - Velocity command components
figure;
plot(t, vy, 'DisplayName', 'Washout velocity');
hold on;
plot(t, K_self*v_self, 'DisplayName', 'Self-centering contribution');
plot(t, K_push*v_push, 'DisplayName', 'Pushback contribution');
plot(t, v_sum, 'DisplayName', 'Combined command');
hold off;
grid on;
xlabel('Time [s]');
ylabel('Velocity Command');
title('Platform Velocity Command Components');
legend('Location', 'best');


% Figure 6 - Motor speed
figure;
plot(t, w_motor);
grid on;
xlabel('Time [s]');
ylabel('Motor Speed [rad/s]');
title('Motor Speed');


% Figure 7 - Motor torque
figure;
plot(t, T_cmd);
grid on;
xlabel('Time [s]');
ylabel('Motor Torque [N·m]');
title('Commanded Motor Torque');


% Figure 8 - Motor controller current
figure;
plot(t, I_cmd);
grid on;
xlabel('Time [s]');
ylabel('Motor Current [A]');
title('AKD Controller Current');


% Figure 9 - Motor reference vs actual speed
figure;
plot(t, w_ref, 'DisplayName', 'Motor speed reference');
hold on;
plot(t, w_motor, 'DisplayName', 'Actual motor speed');
hold off;
grid on;
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title('Motor Speed Tracking');
legend('Location', 'best');

%% ------------------------------------------------------------------------
% Local functions
% -------------------------------------------------------------------------

function vx_selfcentering = self_centering_velocity(x)

% Self-centering velocity
x_neutral = 0.007;       % m
deadband = 0.0005;       % m
Kp = 0.15;               % 1/s
v_max = 0.0004;          % m/s

error = x_neutral - x;

if abs(error) <= deadband
    vx_selfcentering = 0;
else
    vx_selfcentering = Kp * error;
    vx_selfcentering = min(max(vx_selfcentering, ...
                               -v_max), v_max);
end

end


function vx_pushback = pushback_velocity(x)

% Soft-limit push-back velocity
x_soft_minus = -0.01585;     % m
x_soft_plus  =  0.01615;     % m
KpSoft = 20;                 % 1/s
v_max = 0.00075;             % m/s

vx_pushback = 0;

if x > x_soft_plus

    error = x - x_soft_plus;
    vx_pushback = -KpSoft * error;

elseif x < x_soft_minus

    error = x_soft_minus - x;
    vx_pushback = KpSoft * error;

end

vx_pushback = min(max(vx_pushback, -v_max), v_max);

end


function [x, v] = motion_platform(theta, omega)

% Motion Platform Model
Lc = 0.050;
Lr = 0.075;
H  = 0.080;

% Mechanism centre
xc0 = Lc*sin(0);
yc0 = Lc*cos(0);

arg0 = Lr^2 - (H - yc0)^2;
xCentre = xc0 - sqrt(arg0);

% Crank pin
xc = Lc*sin(theta);
yc = Lc*cos(theta);

% Slider
arg = Lr^2 - (H - yc)^2;
arg = max(arg, 0);

term = sqrt(arg);
xs = xc - term;

% Position relative to mechanism centre
x = xs - xCentre;

% dx/dtheta
dx_dtheta = ...
    Lc*cos(theta) + ...
    (Lc*sin(theta) * ...
    (H - Lc*cos(theta))) / max(term, 1e-9);

% Platform velocity
v = dx_dtheta * omega;

end
