function xk1 = myStateTransitionFcn(xk, uk)
%#codegen
% Discrete-time state transition using RK4

% ---- Parameters ----
config = localConfig(); 
Ts = config.sim.var.time_step;

payloadComp = config.sim.mode.controllerConfig.payloadCompensation;
m_ext = config.stateObservability.extKalFil.payload_mass;
W = [0;-9.81*m_ext];

if payloadComp == ControllerPayloadCompensation.NotActive
    % ---- RK4 integration ----
    % dot_x = f(x,u)
    k1 = f(xk,                uk, config);
    k2 = f(xk + 0.5*Ts*k1,     uk, config);
    k3 = f(xk + 0.5*Ts*k2,     uk, config);
    k4 = f(xk + Ts*k3,         uk, config);
elseif payloadComp == ControllerPayloadCompensation.Active
    % ---- RK4 integration ----
    % dot_x = f(x,u)
    k1 = f_payload(xk,                uk, W, config);
    k2 = f_payload(xk + 0.5*Ts*k1,     uk, W, config);
    k3 = f_payload(xk + 0.5*Ts*k2,     uk, W, config);
    k4 = f_payload(xk + Ts*k3,         uk, W, config);
else
    error("Invalid Payload Compensation value");
end

xk1 = xk + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
