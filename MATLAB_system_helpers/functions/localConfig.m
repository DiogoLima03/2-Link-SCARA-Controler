function config = localConfig()
% Auto-generated from workspace variable 'config' on 28-Jan-2026 08:05:54
% THIS FILE IS AUTO-GENERATED. DO NOT EDIT. Generated from './MATLAB_system_helpers/functions/generate_localConfig_from_workspace.m'

% robot params
config.robot.geom.l1  = 0.59999999999999998;
config.robot.geom.l2  = 0.59999999999999998;
config.robot.geom.lc1 = 0.29999999999999999;
config.robot.geom.lc2 = 0.29999999999999999;

config.robot.dyn.m1 = 2.6136000000000008;
config.robot.dyn.m2 = 2.6136000000000008;
config.robot.dyn.I1 = 0.079736580000000001;
config.robot.dyn.I2 = 0.079736580000000001;
config.robot.dyn.D1 = 0.10000000000000001;
config.robot.dyn.D2 = 0.10000000000000001;

config.robot.payload.mass = 2.7999999999999998;
config.robot.gravity.alpha = 0;

% simulation step
config.sim.var.time_step = 0.0040000000000000001;

% controller configuration
config.sim.mode.controllerConfig.action = ControllerAction.PID_zeta_omega;
config.sim.mode.controllerConfig.payloadCompensation = ControllerPayloadCompensation.Active;

% EKF / state observability configuration
config.stateObservability.extKalFil.payload_mass = 0.5;

end
