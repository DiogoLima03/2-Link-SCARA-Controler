%% Robot Parameters used in the Controller

% --- Link geometry ---
config.robot.geom.l1  = 0.6;                    % [m]
config.robot.geom.l2  = 0.6;                    % [m]
config.robot.geom.lc1 = config.robot.geom.l1/2; % [m]
config.robot.geom.lc2 = config.robot.geom.l2/2; % [m]

% --- Cross-section geometry ---
config.robot.cs.b_outer = 0.060;              % [m]
config.robot.cs.h       = 0.060;              % [m]
config.robot.cs.t       = 5e-3;               % [m]
config.robot.cs.L       = 0.6;                % [m]
config.robot.cs.rho     = 3960;               % [kg/m^3]

% --- Mass and inertia ---
[config.robot.dyn.m1, config.robot.dyn.I1] = RectangleHollowLink( ...
    config.robot.cs.b_outer, ...
    config.robot.cs.h, ...
    config.robot.cs.t, ...
    config.robot.cs.t, ...
    config.robot.cs.L, ...
    config.robot.cs.rho );

config.robot.dyn.m2 = config.robot.dyn.m1;    % [kg]
config.robot.dyn.I2 = config.robot.dyn.I1;    % [kg·m^2]

% --- Joint damping ---
config.robot.dyn.D1 = 0.1;                    % [kg·m^2/s]
config.robot.dyn.D2 = 0.1;                    % [kg·m^2/s]

% --- External payload that the controller assumes ---
config.contrl.non_linear_fdbck_lin.payload.mass = 0.5;

% --- Gravity configuration ---
config.robot.gravity.alpha = 0;               % [rad]

%% Robot Parameters used to simulate the plant

% --- Link geometry ---
config.robot_plant.geom.l1  = 0.6;                    % [m]
config.robot_plant.geom.l2  = 0.6;                    % [m]
config.robot_plant.geom.lc1 = config.robot_plant.geom.l1/2; % [m]
config.robot_plant.geom.lc2 = config.robot_plant.geom.l2/2; % [m]

% --- Cross-section geometry ---
config.robot_plant.cs.b_outer = 0.060;              % [m]
config.robot_plant.cs.h       = 0.060;              % [m]
config.robot_plant.cs.t       = 5e-3;               % [m]
config.robot_plant.cs.L       = 0.6;                % [m]
config.robot_plant.cs.rho     = 3960;               % [kg/m^3]

% --- Mass and inertia ---
[config.robot_plant.dyn.m1, config.robot_plant.dyn.I1] = RectangleHollowLink( ...
    config.robot_plant.cs.b_outer, ...
    config.robot_plant.cs.h, ...
    config.robot_plant.cs.t, ...
    config.robot_plant.cs.t, ...
    config.robot_plant.cs.L, ...
    config.robot_plant.cs.rho );

config.robot_plant.dyn.m2 = config.robot.dyn.m1;    % [kg]
config.robot_plant.dyn.I2 = config.robot.dyn.I1;    % [kg·m^2]

% --- Joint damping ---
config.robot_plant.dyn.D1 = 0.1;                    % [kg·m^2/s]
config.robot_plant.dyn.D2 = 0.1;                    % [kg·m^2/s]

% --- External payload ---
config.robot.payload.mass = 2.8;                    % [kg]

% --- Gravity configuration ---
config.robot_plant.gravity.alpha = 0;               % [rad]

%% Input / Motor Saturation
config.robot.motor.saturation.joint_1 = 80;   % [Nm]
config.robot.motor.saturation.joint_2 = 25;   % [Nm]

%% Reference Generation

% Regulation 
config.refGeneration.regulation.pos_d = [0;  % q (rad)
                                         0;  % q (rad)
                                         0;  % dot_q (rad/s)
                                         0;  % dot_q (rad/s)
                                         0;  % ddot_q (rad/s^2)
                                         0]; % ddot_q (rad/s^2)
% Initial conditions

% Robot initial angular velocity
config.refGeneration.regulation.init_cond.ang_vel = [ 0;
                                                      0];
% Robot initial angular position
config.refGeneration.regulation.init_cond.ang_pos = [ -pi/2;
                                                      -pi];

config.refGeneration.regulation.init_cond.states_vec = [config.refGeneration.regulation.init_cond.ang_pos;
                                                        config.refGeneration.regulation.init_cond.ang_vel];

% Trajectory

% max vel and acc that the robot can reach while performing the trajectory
config.refGeneration.trajectory.max_vel = 25; % [m/s] do not touch
config.refGeneration.trajectory.max_acc = 5; % [m/s^2] do not touch

% time scale
config.refGeneration.trajectory.time_scale = 1.2;   % do not touch

% alpha 
config.refGeneration.trajectory.alpha = 1;   % 0.5 centripetal Catmull–Rom

% waypoints
config.refGeneration.trajectory.P0 = [ 466.50e-3; -631.50e-3];
config.refGeneration.trajectory.P1 = [ 466.50e-3; -481.50e-3];
config.refGeneration.trajectory.P2 = [992e-3; 315e-3];
config.refGeneration.trajectory.P3 = [ 848.53e-3;  848.53e-3];
config.refGeneration.trajectory.P4 = [315e-3; 992e-3];
config.refGeneration.trajectory.P5 = [-466.50e-3;  150e-3];
config.refGeneration.trajectory.P6 = [-466.50e-3;    0     ];

config.refGeneration.trajectory.W = [config.refGeneration.trajectory.P0, ...
                                     config.refGeneration.trajectory.P1, ...
                                     config.refGeneration.trajectory.P2, ...
                                     config.refGeneration.trajectory.P3, ...
                                     config.refGeneration.trajectory.P4, ...
                                     config.refGeneration.trajectory.P5, ...
                                     config.refGeneration.trajectory.P6];


% init Position / Velocity / Acceleration
config.refGeneration.trajectory.initPosVelAcc = [466.50e-3;    % pos x
                                                 -631.50e-3;   % pos y
                                                 0;            % vel x
                                                 0;            % vel y
                                                 0;            % acc x
                                                 0];          % acc y

% final Pos
config.refGeneration.trajectory.finalPosVelAcc = [-466.50e-3;   % pos x
                                                  0;            % pos y
                                                  0;            % vel x
                                                  0;            % vel y
                                                  0;            % acc x
                                                  0];           % acc y

%% simulation variables
config.sim.var.sim_time = 3;
config.sim.var.time_step = 0.004;
config.sim.var.freq = 1/config.sim.var.time_step;

% Initial conditions
if config.sim.mode.refGeneration == RefGeneration.Regulation
    config.sim.var.init_cond.ang_pos = config.refGeneration.regulation.init_cond.ang_pos;
    config.sim.var.init_cond.ang_vel = config.refGeneration.regulation.init_cond.ang_vel;

elseif config.sim.mode.refGeneration == RefGeneration.Trajectory
    [config.sim.var.init_cond.ang_pos, config.sim.var.init_cond.ang_vel] = inverse_kin(config.refGeneration.trajectory.initPosVelAcc(1:2), config.refGeneration.trajectory.initPosVelAcc(3:4), [], config, "down");
    
else
    error("Reference Generation mode invalid.");
end

config.sim.var.init_cond.states_vec = [config.sim.var.init_cond.ang_pos;
                                       config.sim.var.init_cond.ang_vel];

%% Non-Linear Controller params (Feedback linearisation)

% No Payload Compensation =================================================
%   PD_zeta_omega (butterworth equations) ---------------------------------

% selecting different gains if its regulation or tragectory beacause it
% would be to fast for regulation (slower is easier to see)
if config.sim.mode.refGeneration == RefGeneration.Regulation
    config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.settling_time.joint_1 = 1.5;
    config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.settling_time.joint_2 = 1.5;
elseif config.sim.mode.refGeneration == RefGeneration.Trajectory
    config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.settling_time.joint_1 = 0.3;  % considering a 2% settling time (advised [0.15, 0.5])
    config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.settling_time.joint_2 = 0.3; %0.3
else
    error("Infalid trag generation simulation selection.");
end

%       joint 1
config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.zeta.joint_1 = 0.7;               % damping ratio - 0.7=good compromise, small overshot (=~5%)
config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.omega.joint_1 = 4/(config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.zeta.joint_1*config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.settling_time.joint_1);% natural frequency
%       joint 2
config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.zeta.joint_2 = 0.7;               % damping ratio - 0.7=good compromise, small overshot (=~5%)
config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.omega.joint_2 = 4/(config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.zeta.joint_2*config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.settling_time.joint_2);% natural frequency
%       k_p
config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.gains.k_p = diag([config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.omega.joint_1^2;...
                                                                              config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.omega.joint_2^2]);
%       k_d 
config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.gains.k_d = diag([2*config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.zeta.joint_1*config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.omega.joint_1;...
                                                                              2*config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.zeta.joint_2*config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.omega.joint_2]);
%   PD_LQR ----------------------------------------------------------------
if config.sim.mode.refGeneration == RefGeneration.Regulation
    config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_p = ...
          -[ -299.5681    0.0000
              0.0000     -299.5681];
    config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_d = ...
          -[-26.2465    0.0000
             0.0000  -26.2465];
elseif config.sim.mode.refGeneration == RefGeneration.Trajectory
    config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_p = ...
          -[-9.9088   -0.0000
           -0.0000   -9.8341];
    config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_d = ...
          -[-4.5606    0.0000
           -0.0000   -8.2476];    
else
    error("Infalid trag generation simulation selection.");
end


%   PID_zeta_omega --------------------------------------------------------
config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_p = config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.gains.k_p;
config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_d = config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.gains.k_d;
config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_i = diag([1500; 1500]);
%   PID_LQR ---------------------------------------------------------------
config.contrl.non_linear_fdbck_lin.no_payload.PID_LQR.gains.k_p = config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_p;
config.contrl.non_linear_fdbck_lin.no_payload.PID_LQR.gains.k_d = config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_d;
config.contrl.non_linear_fdbck_lin.no_payload.PID_LQR.gains.k_i = config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_i;


% Width Payload Compensation ==============================================
% mass to be compensated
config.contrl.non_linear_fdbck_lin.payload.mass = 0.5;


%   PD_zeta_omega ---------------------------------------------------------
config.contrl.non_linear_fdbck_lin.payload.PD_zeta_omega.gains.k_p = config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.gains.k_p;
config.contrl.non_linear_fdbck_lin.payload.PD_zeta_omega.gains.k_d = config.contrl.non_linear_fdbck_lin.no_payload.PD_zeta_omega.gains.k_d;
%   PD_LQR ----------------------------------------------------------------
config.contrl.non_linear_fdbck_lin.payload.PD_LQR.gains.k_p = config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_p;
config.contrl.non_linear_fdbck_lin.payload.PD_LQR.gains.k_d = config.contrl.non_linear_fdbck_lin.no_payload.PD_LQR.gains.k_d;
%   PID_zeta_omega --------------------------------------------------------
config.contrl.non_linear_fdbck_lin.payload.PID_zeta_omega.gains.k_p = config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_p;
config.contrl.non_linear_fdbck_lin.payload.PID_zeta_omega.gains.k_d = config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_d;
config.contrl.non_linear_fdbck_lin.payload.PID_zeta_omega.gains.k_i = config.contrl.non_linear_fdbck_lin.no_payload.PID_zeta_omega.gains.k_i;
%   PID_LQR ---------------------------------------------------------------
config.contrl.non_linear_fdbck_lin.payload.PID_LQR.gains.k_p = config.contrl.non_linear_fdbck_lin.no_payload.PID_LQR.gains.k_p;
config.contrl.non_linear_fdbck_lin.payload.PID_LQR.gains.k_d = config.contrl.non_linear_fdbck_lin.no_payload.PID_LQR.gains.k_d;
config.contrl.non_linear_fdbck_lin.payload.PID_LQR.gains.k_i = config.contrl.non_linear_fdbck_lin.no_payload.PID_LQR.gains.k_i;

% I controller params
% time step
config.contrl.non_linear_fdbck_lin.param.I_controller.intStep = config.sim.var.time_step;
% clamoing e_i_prev (max)
config.contrl.non_linear_fdbck_lin.param.I_controller.clamp_max = [100; 100];

%% Measurement Noise

config.disturbances.measurements.noise.encoders.timeStep = config.sim.var.time_step; % [s]
%config.disturbances.measurements.noise.encoders.power_aux = 1*config.disturbances.measurements.noise.encoders.timeStep;

% Encoder:
% noise standard deviation. Some values we can follow (chat GPT) after i saw that maybe this noise values are to low it should be *10 of what it said:
% good encoder: [0.0005, 0.001]
% moderate: [0.001, 0.003]
% noisy : 0.005
config.disturbances.measurements.noise.encoder_1.stdrdev = 0.02; % [rad] 
config.disturbances.measurements.noise.encoder_2.stdrdev = 0.02; % [rad] 

% noise variance
config.disturbances.measurements.noise.encoder_1.variance = config.disturbances.measurements.noise.encoder_1.stdrdev^2;
config.disturbances.measurements.noise.encoder_2.variance = config.disturbances.measurements.noise.encoder_2.stdrdev^2;

% noise power (aprox)
config.disturbances.measurements.noise.encoder_1.power = config.disturbances.measurements.noise.encoder_1.variance*config.disturbances.measurements.noise.encoders.timeStep;
config.disturbances.measurements.noise.encoder_2.power = config.disturbances.measurements.noise.encoder_2.variance*config.disturbances.measurements.noise.encoders.timeStep;

% noise correlation
config.disturbances.measurements.noise.encoders.correlation = 0;

% for all sensors we have:
config.disturbances.measurements.noise.covariance = [config.disturbances.measurements.noise.encoder_1.variance, config.disturbances.measurements.noise.encoders.correlation*config.disturbances.measurements.noise.encoder_1.stdrdev*config.disturbances.measurements.noise.encoder_2.stdrdev;
                                                     config.disturbances.measurements.noise.encoders.correlation*config.disturbances.measurements.noise.encoder_1.stdrdev*config.disturbances.measurements.noise.encoder_2.stdrdev, config.disturbances.measurements.noise.encoder_2.variance];

% Ang Vel - Some noise just to demonstrate
config.disturbances.measurements.noise.ang_vel_1.stdrdev = 0.002; 
config.disturbances.measurements.noise.ang_vel_2.stdrdev = 0.002; 
config.disturbances.measurements.noise.ang_vel_1.variance = config.disturbances.measurements.noise.ang_vel_1.stdrdev^2;
config.disturbances.measurements.noise.ang_vel_2.variance = config.disturbances.measurements.noise.ang_vel_2.stdrdev^2;

% noise power (aprox)
config.disturbances.measurements.noise.ang_vel_1.power = config.disturbances.measurements.noise.ang_vel_1.variance*config.disturbances.measurements.noise.encoders.timeStep;
config.disturbances.measurements.noise.ang_vel_2.power = config.disturbances.measurements.noise.ang_vel_2.variance*config.disturbances.measurements.noise.encoders.timeStep;

%% External Disturbance Forces / Model Uncertainties

% Gaussian Noise
config.disturbances.external.noise.timeStep = config.sim.var.time_step; % [s]

config.disturbances.external.noise.tau_1.stdrdev = 0.25; % [Nm] 
config.disturbances.external.noise.tau_2.stdrdev = 0.25; % [Nm] 

% noise variance
config.disturbances.external.noise.tau_1.variance = config.disturbances.external.noise.tau_1.stdrdev^2;
config.disturbances.external.noise.tau_2.variance = config.disturbances.external.noise.tau_2.stdrdev^2;

% noise power (aprox)
config.disturbances.external.noise.tau_1.power = config.disturbances.external.noise.tau_1.variance*config.disturbances.external.noise.timeStep;
config.disturbances.external.noise.tau_2.power = config.disturbances.external.noise.tau_2.variance*config.disturbances.external.noise.timeStep;

% noise correlation
config.disturbances.external.noise.correlation = 0;

% Noise covariance matrix
config.disturbances.external.noise.covariance = [ ...
    config.disturbances.external.noise.tau_1.variance, ...
    config.disturbances.external.noise.correlation * ...
        config.disturbances.external.noise.tau_1.stdrdev * ...
        config.disturbances.external.noise.tau_2.stdrdev; ...
    config.disturbances.external.noise.correlation * ...
        config.disturbances.external.noise.tau_1.stdrdev * ...
        config.disturbances.external.noise.tau_2.stdrdev, ...
    config.disturbances.external.noise.tau_2.variance ];

% Constant Force
config.disturbances.external.constForce = [10;
                                           10]; % Nm

%% State Reconstruction: Derivative with Low Pass Filter

% low pass filter freq
config.stateObservability.stateReconstruction.DevLowPassFilt.cutoffFreq = 3; % 5[Hz] 
config.stateObservability.stateReconstruction.DevLowPassFilt.angVelocity = 2*pi*config.stateObservability.stateReconstruction.DevLowPassFilt.cutoffFreq; % [rad/s]

%% State Reconstruction: Dirty Derivative

% low pass fikter freq
config.stateObservability.stateReconstruction.DirtyDev.cutoffFreq = 3; % [Hz]
config.stateObservability.stateReconstruction.DirtyDev.timeConst = 1/(2*pi*config.stateObservability.stateReconstruction.DirtyDev.cutoffFreq); % [s]

%% Extended Kalman-Filter

config.stateObservability.extKalFil.payload_mass = config.contrl.non_linear_fdbck_lin.payload.mass; % [Kg]

% initial covariances - how sure you are that your real initial conditions
% macth the conditions specified in the observer (low because its a simulation)
config.stateObservability.extKalFil.init_state_cov = ...
diag([
    1e-6;
    1e-6;
    1e-2;
    1e-2
]);

% -------------------------------------------------------------------------
% Process noise covariance Q
% -------------------------------------------------------------------------
% Represents uncertainty in the system model (unmodeled dynamics,
% disturbances, parameter errors, numerical integration errors).
%
% lower  - Assumes a highly accurate model.
%          EKF trusts the prediction model more than the measurements.
%          Slower adaptation to disturbances or modeling errors.
%          Risk of filter divergence if the model is incorrect.
%
% higher - Assumes significant model uncertainty.
%          EKF relies more on measurements.
%          Faster response to disturbances and unmodeled dynamics.
%          Increased estimation noise and less smooth state estimates.
config.stateObservability.extKalFil.process_noise = ...
diag([1e-16, 1e-16, 2e-12, 9e-6]); %[1e-16, 1e-16, 2e-12, 5e-4][1e-8, 1e-8, 2e-8, 5e-4]

% -------------------------------------------------------------------------
% Measurement noise covariance R
% -------------------------------------------------------------------------
% Represents sensor uncertainty (quantization, noise, bias, latency).
%
% lower  - Assumes high-quality, reliable sensors.
%          EKF trusts measurements strongly.
%          Faster correction of state estimates.
%          Risk of injecting measurement noise into the estimate.
%
% higher - Assumes noisy or unreliable sensors.
%          EKF trusts the model more than the measurements.
%          Smoother estimates but slower correction.
%          Risk of lag and reduced tracking accuracy.
config.stateObservability.extKalFil.meas_noise = diag([0.0001, 0.0001]); % 0.0004

% Key practical insight (important for tuning):
% 
% Increasing Q relative to R makes the EKF behave more like a high-gain observer, fast but noisy.
% 
% Decreasing Q relative to R makes the EKF behave more like a model-based predictor, smooth but potentially inaccurate.
% 
% Correct tuning is not about "small vs large" in absolute terms, but about the ratio between Q and R.