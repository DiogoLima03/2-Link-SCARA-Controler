generate_localConfig_from_workspace_func('config');

function generate_localConfig_from_workspace_func(varargin)
%GENERATE_LOCALCONFIG_FROM_WORKSPACE Create/overwrite localConfig.m based on a config struct in workspace.
%
% Usage:
%   generate_localConfig_from_workspace();                 % reads 'config' from base workspace
%   generate_localConfig_from_workspace('myConfigName');   % reads that variable name
%
% Output:
%   Writes ./simulation_helper_functions/linear_system/localConfig.m

    if nargin < 1 || isempty(varargin{1})
        configVarName = 'config';
    else
        configVarName = varargin{1};
    end

    % Read config from base workspace (NOT codegen, intended for setup time)
    cfg = evalin('base', configVarName);

    % Extract the fields you want (fail fast if missing)
    req = @(s, f) assert(isfield(s, f), 'Missing field: %s', f);

    req(cfg, 'robot'); req(cfg.robot, 'geom'); req(cfg.robot, 'dyn');
    req(cfg.robot, 'payload'); req(cfg.robot, 'gravity');
    req(cfg, 'sim'); req(cfg.sim, 'var');

    % Extracting config.stateObservability.extKalFil.payload_mass
    req(cfg, 'stateObservability');
    req(cfg.stateObservability, 'extKalFil');
    req(cfg.stateObservability.extKalFil, 'payload_mass');
    
    payload_mass_kf = cfg.stateObservability.extKalFil.payload_mass;
    
    assert(isnumeric(payload_mass_kf) && isscalar(payload_mass_kf), ...
        'stateObservability.extKalFil.payload_mass must be a numeric scalar');
    

    l1    = cfg.robot.geom.l1;
    l2    = cfg.robot.geom.l2;
    lc1   = cfg.robot.geom.lc1;
    lc2   = cfg.robot.geom.lc2;

    m1    = cfg.robot.dyn.m1;
    m2    = cfg.robot.dyn.m2;
    I1    = cfg.robot.dyn.I1;
    I2    = cfg.robot.dyn.I2;
    D1    = cfg.robot.dyn.D1;
    D2    = cfg.robot.dyn.D2;
    
    m_ext = cfg.robot.payload.mass;
    alpha = cfg.robot.gravity.alpha;

    Ts    = cfg.sim.var.time_step;

    actionEnum = cfg.sim.mode.controllerConfig.action;
    payloadEnum = cfg.sim.mode.controllerConfig.payloadCompensation;
    
    actionEnumStr  = char(actionEnum);         
    actionEnumCls  = class(actionEnum);         % "ControllerAction"
    
    payloadEnumStr = char(payloadEnum);         
    payloadEnumCls = class(payloadEnum);        % "PayloadCompensation"

    % Write the function file with numeric literals
    fid = fopen('localConfig.m', 'w');
    assert(fid ~= -1, 'Could not open for writing: %s', 'localConfig.m');
    c = onCleanup(@() fclose(fid));

    fprintf(fid, "function config = localConfig()\n");
    fprintf(fid, "%% Auto-generated from workspace variable '%s' on %s\n", configVarName, datestr(now));
    fprintf(fid, "%% THIS FILE IS AUTO-GENERATED. DO NOT EDIT. Generated from './MATLAB_system_helpers/functions/generate_localConfig_from_workspace.m'\n");
    fprintf(fid, "\n");

    fprintf(fid, "%% robot params\n");
    fprintf(fid, "config.robot.geom.l1  = %.17g;\n", l1);
    fprintf(fid, "config.robot.geom.l2  = %.17g;\n", l2);
    fprintf(fid, "config.robot.geom.lc1 = %.17g;\n", lc1);
    fprintf(fid, "config.robot.geom.lc2 = %.17g;\n", lc2);
    fprintf(fid, "\n");

    fprintf(fid, "config.robot.dyn.m1 = %.17g;\n", m1);
    fprintf(fid, "config.robot.dyn.m2 = %.17g;\n", m2);
    fprintf(fid, "config.robot.dyn.I1 = %.17g;\n", I1);
    fprintf(fid, "config.robot.dyn.I2 = %.17g;\n", I2);
    fprintf(fid, "config.robot.dyn.D1 = %.17g;\n", D1);
    fprintf(fid, "config.robot.dyn.D2 = %.17g;\n", D2);
    fprintf(fid, "\n");

    fprintf(fid, "config.robot.payload.mass = %.17g;\n", m_ext);
    fprintf(fid, "config.robot.gravity.alpha = %.17g;\n", alpha);
    fprintf(fid, "\n");

    fprintf(fid, "%% simulation step\n");
    fprintf(fid, "config.sim.var.time_step = %.17g;\n", Ts);

    fprintf(fid, "\n%% controller configuration\n");
    fprintf(fid, "config.sim.mode.controllerConfig.action = %s.%s;\n", ...
            actionEnumCls, actionEnumStr);
    
    fprintf(fid, "config.sim.mode.controllerConfig.payloadCompensation = %s.%s;\n", ...
            payloadEnumCls, payloadEnumStr);

    fprintf(fid, "\n%% EKF / state observability configuration\n");
    fprintf(fid, "config.stateObservability.extKalFil.payload_mass = %.17g;\n", ...
        payload_mass_kf);


    fprintf(fid, "\nend\n");
end
