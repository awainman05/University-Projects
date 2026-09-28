%% SystemModelWashout_converted.m
% MATLAB-script equivalent of SystemModelWashout.slx
%
% This file reproduces the active signal path in the uploaded Simulink
% model without requiring Simulink.
%
% INPUTS
%   Ay : lateral acceleration input [g]
%   Ax : longitudinal acceleration input [g] (currently unused by the
%        active washout path in the uploaded model)
%   Az : vertical acceleration input [g] (currently unused by the active
%        washout path in the uploaded model)
%
% The original model uses From Workspace blocks named Ay, Ax and Az.
% For a direct test, define those vectors before running this script.
%
% Example:
%   Ts   = 0.001;
%   T_end = 10;
%   t = (0:Ts:T_end).';
%   Ay = 0.1*sin(2*pi*0.5*t);
%   Ax = zeros(size(t));
%   Az = zeros(size(t));
%   run('SystemModelWashout_converted.m')
%
% The main outputs are:
%   v_platformcmd : final platform command
%   vx_limited    : final control-algorithm output
%   vy            : washout lateral velocity component
%   x             : platform position [m]
%   v             : platform velocity [m/s]
%   w_motor       : motor speed [rad/s]
%   theta_motor   : motor shaft angle [rad]
%   T_cmd         : commanded motor torque [N*m]

%% ------------------------------------------------------------------------
% Simulation settings and input data
% -------------------------------------------------------------------------

Ts = 0.001;                 % Model sample time [s]

% Load lap data
load("C:\Users\alexw\Uni Work\Cranfield\IRP\Data\ks_silverstone_&_rss_formula_hybrid_2022_&_cranfieldmotorsports_&_stint_23.mat");

% Original time vector
t_data = Lap_Time.Value(:);

% Acceleration data
Ay_data = CG_Accel_Longitudinal.Value;
Ax_data = CG_Accel_Lateral.Value;
Az_data = CG_Accel_Vertical.Value;

% Convert all acceleration signals to column vectors
Ay_data = Ay_data(:);
Ax_data = Ax_data(:);
Az_data = Az_data(:);

% Remove duplicate time values
[t_data, unique_idx] = unique(t_data, 'stable');

% Keep the acceleration samples corresponding to the unique times
Ay_data = Ay_data(unique_idx);
Ax_data = Ax_data(unique_idx);
Az_data = Az_data(unique_idx);

% Simulation end time
T_end = t_data(end);

% Create simulation time vector
t = (0:Ts:T_end).';

% Make sure final time is included
if t(end) < T_end
    t = [t; T_end];
end

Nsim = numel(t);

% Interpolate input signals onto the 1 ms simulation time grid
Ay = interp1(t_data, Ay_data, t, 'linear', 'extrap');
Ax = interp1(t_data, Ax_data, t, 'linear', 'extrap');
Az = interp1(t_data, Az_data, t, 'linear', 'extrap');

% Make sure outputs are column vectors
Ay = Ay(:);
Ax = Ax(:);
Az = Az(:);

%% ------------------------------------------------------------------------
% Allocate signals
% -------------------------------------------------------------------------

% Control algorithm
Ay_limited     = zeros(Nsim,1);
Ay_filtered    = zeros(Nsim,1);
W_ay           = zeros(Nsim,1);
W_ay_scaled    = zeros(Nsim,1);
vy             = zeros(Nsim,1);
v_self         = zeros(Nsim,1);
v_push         = zeros(Nsim,1);
v_sum          = zeros(Nsim,1);
vx_limited     = zeros(Nsim,1);
v_platformcmd  = zeros(Nsim,1);

% Controller / motor
w_feedback     = zeros(Nsim,1);
T_cmd          = zeros(Nsim,1);
I_cmd          = zeros(Nsim,1);
w_motor        = zeros(Nsim,1);
theta_motor    = zeros(Nsim,1);

% Harmonic gear / platform
theta_crank    = zeros(Nsim,1);
omega_crank    = zeros(Nsim,1);
x              = zeros(Nsim,1);
v              = zeros(Nsim,1);

% Internal states
integ_PI       = 0;
integrator_washout = 0;
transfer_x1    = 0;
transfer_y1    = 0;
rate_limiter_y = 0;
w_delay        = 0;

%% ------------------------------------------------------------------------
% Initial motor state
% -------------------------------------------------------------------------

% From Motor: LynxDrive-C MATLAB Function
J_motor = 3.12e-4;          % kg*m^2
B_motor = 0.076;            % N*m*s/rad
w_max = 502.65;             % rad/s

N_gear = 100;

Lc = 0.050;                 % m
Lr = 0.075;                 % m
H  = 0.080;                 % m
m_load = 100;               % kg

theta_crank_min = deg2rad(-21.9632);
theta_crank_max = deg2rad(18.4462);
theta_motor_min = N_gear * theta_crank_min;
theta_motor_max = N_gear * theta_crank_max;

w_motor(1) = 0;
theta_motor(1) = N_gear * deg2rad(7.81);

% Controller parameters
washout_to_radps = 500;
Kp_controller = 0.363;
Ki_controller = 24.972;
Kt = 0.58;
I_peak = 18.0;

% Control-algorithm parameters
K_washout = 0.05;
K_self = 5.5;
K_push = 10;

% Active discrete transfer function:
% numerator   = [0.998744 -0.998744]
% denominator = [1 -0.997488]
b0 = 0.998744;
b1 = -0.998744;
a1 = -0.997488;

% Active output rate limiter:
% rising slew limit  = +5
% falling slew limit  = -5
% with Ts = 0.001 s, maximum output change per sample is +/-0.005.
rise_per_sample = 5 * Ts;
fall_per_sample = -5 * Ts;

%% ------------------------------------------------------------------------
% Main simulation loop
% -------------------------------------------------------------------------

for k = 1:Nsim

    %% ===== PLATFORM POSITION / VELOCITY =====
    % Motor shaft -> harmonic drive -> crank
    theta_crank(k) = theta_motor(k) / N_gear;
    omega_crank(k) = w_motor(k) / N_gear;

    % Motion Platform MATLAB Function
    [x(k), v(k)] = motion_platform(theta_crank(k), omega_crank(k));

    % The Motion Platform subsystem has a +/-0.17 m saturation.
    % This is much larger than the intended +/-17 mm travel and is
    % therefore retained exactly as in the Simulink model.
    x(k) = min(max(x(k), -0.17), 0.17);

    %% ===== CONTROL ALGORITHM =====

    % Ay enters Saturation4: +/- 6 g
    Ay_limited(k) = min(max(Ay(k), -6), 6);

    % MATLAB Function block "MATLAB Function" is COMMENTED THROUGH
    % in the uploaded .slx. Therefore its active behavior is a direct
    % pass-through, not the threshold function stored in its Stateflow
    % chart.
    Ay_filtered(k) = Ay_limited(k);

    % Discrete Transfer Fcn1
    %
    % y[k] = -a1*y[k-1] + b0*u[k] + b1*u[k-1]
    %
    % Because the denominator is [1 -0.997488], -a1 = +0.997488.
    W_ay(k) = -a1 * transfer_y1 + ...
              b0 * Ay_filtered(k) + ...
              b1 * transfer_x1;

    transfer_x1 = Ay_filtered(k);
    transfer_y1 = W_ay(k);

    % Gain = 0.05
    W_ay_scaled(k) = K_washout * W_ay(k);

    % Discrete-Time Integrator2, inherited sample time = 0.001 s
    integrator_washout = integrator_washout + Ts * W_ay_scaled(k);
    vy(k) = integrator_washout;

    % Position-based self-centering velocity
    v_self(k) = self_centering_velocity(x(k));

    % Position-based pushback velocity
    v_push(k) = pushback_velocity(x(k));

    % Gains 5 and 10, followed by the 3-input Sum block
    v_sum(k) = vy(k) + K_self*v_self(k) + K_push*v_push(k);

    % Rate Limiter1
    if k == 1
        vx_rate_limited = rate_limiter_y;
    else
        delta = v_sum(k) - rate_limiter_y;
        if delta > rise_per_sample
            delta = rise_per_sample;
        elseif delta < fall_per_sample
            delta = fall_per_sample;
        end
        vx_rate_limited = rate_limiter_y + delta;
    end
    rate_limiter_y = vx_rate_limited;

    % Saturation1: +/-0.3
    vx_limited(k) = min(max(vx_rate_limited, -0.3), 0.3);

    v_platformcmd(k) = vx_limited(k);

    %% ===== AKD MOTOR CONTROLLER =====
    %
    % The root model contains a Unit Delay on motor velocity before
    % feeding the AKD controller. Thus the controller uses the previous
    % motor-speed sample.

    w_feedback(k) = w_delay;

    w_ref = v_platformcmd(k) * washout_to_radps;
    e = w_ref - w_feedback(k);

    I_unsat = Kp_controller*e + Ki_controller*integ_PI;

    % Current saturation
    I_cmd(k) = min(max(I_unsat, -I_peak), I_peak);

    % Anti-windup exactly as implemented in the MATLAB Function block
    if (abs(I_unsat) < I_peak) || ...
       ((I_unsat > I_peak) && (e < 0)) || ...
       ((I_unsat < -I_peak) && (e > 0))

        integ_PI = integ_PI + e*Ts;
    end

    % Current -> torque
    T_cmd(k) = Kt * I_cmd(k);

    %% ===== MOTOR MODEL =====

    theta_now = theta_motor(k);
    w_now = w_motor(k);

    theta_crank_now = theta_now / N_gear;

    term_squared = Lr^2 - ...
        (H - Lc*cos(theta_crank_now))^2;

    term_squared = max(term_squared, 1e-12);
    term = sqrt(term_squared);

    dx_dtheta = ...
        Lc*cos(theta_crank_now) + ...
        (Lc*sin(theta_crank_now) * ...
        (H - Lc*cos(theta_crank_now))) / term;

    J_load = m_load * dx_dtheta^2 / N_gear^2;
    J_total = J_motor + J_load;

    dw = (T_cmd(k) - B_motor*w_now) / J_total;

    w_new = w_now + Ts*dw;

    % Motor speed limit
    w_new = min(max(w_new, -w_max), w_max);

    % Motor hard-position limits
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

    % Update motor angle
    theta_new = theta_new + w_new*Ts;

    % Store next state
    if k < Nsim
        w_motor(k+1) = w_new;
        theta_motor(k+1) = theta_new;
    end

    % Unit Delay in the root model: motor speed -> delayed feedback
    w_delay = w_new;
end

%% ------------------------------------------------------------------------
% Results
% -------------------------------------------------------------------------

figure(1); % Washout Signals
plot(t, Ay, t, W_ay, t, vy);
grid on;
xlabel('Time [s]');
ylabel('Signal');
legend('A_y', 'W_{ay}', 'v_y', 'Location', 'best');
title('Washout signals');

figure(2); % Platform Position
plot(t, x*1000);
grid on;
xlabel('Time [s]');
ylabel('Platform position [mm]');
title('Platform position');

figure(3); % Platform commanded Velocity
plot(t, v_platformcmd);
grid on;
xlabel('Time [s]');
ylabel('Platform command');
title('Final platform command');

figure(4); % Motor Speed [rad/s]
plot(t, w_motor);
grid on;
xlabel('Time [s]');
ylabel('Motor speed [rad/s]');
title('Motor speed');

%% ------------------------------------------------------------------------
% Local functions
% -------------------------------------------------------------------------

function y = make_column_signal(u, Nsim, name)
    if isscalar(u)
        y = repmat(u, Nsim, 1);
    else
        y = u(:);
        if numel(y) ~= Nsim
            error('%s must contain exactly %d samples, or be a scalar.', ...
                name, Nsim);
        end
    end
end

function y = self_centering_velocity(x)
    % Equivalent to Control Algorithm/Self-Centering Velocity

    x_neutral = 0.015;       % m
    deadband = 0.0005;       % m
    Kp = 0.15;               % 1/s
    v_max = 0.0004;          % m/s

    error = x_neutral - x;

    if abs(error) <= deadband
        y = 0;
    else
        y = Kp * error;
        y = min(max(y, -v_max), v_max);
    end
end

function y = pushback_velocity(x)
    % Equivalent to Control Algorithm/Pushback Velocity

    x_soft_minus = -0.01585; % m
    x_soft_plus  =  0.01615; % m
    KpSoft = 20;              % 1/s
    v_max = 0.00075;          % m/s

    y = 0;

    if x > x_soft_plus
        error = x - x_soft_plus;
        y = -KpSoft * error;
    elseif x < x_soft_minus
        error = x_soft_minus - x;
        y = KpSoft * error;
    end

    y = min(max(y, -v_max), v_max);
end

function [x, v] = motion_platform(theta, omega)
    % Equivalent to Motion Platform MATLAB Function

    Lc = 0.050;
    Lr = 0.075;
    H  = 0.080;

    % Mechanism centre position
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

    % Jacobian
    dx_dtheta = ...
        Lc*cos(theta) + ...
        (Lc*sin(theta)*(H - Lc*cos(theta))) / max(term, 1e-9);

    % Platform velocity
    v = dx_dtheta * omega;
end
