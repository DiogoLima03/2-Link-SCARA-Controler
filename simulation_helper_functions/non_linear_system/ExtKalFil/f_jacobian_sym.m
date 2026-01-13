function Symb_f_jacobian = f_jacobian_sym(q1,q2,dot_q1,dot_q2,u1,u2)

x = [q1, q2, dot_q1, dot_q2];
u = [u1, u2];

% equation generated at 'simulation_helpers_function/linear_system'
config = localConfig();

payloadComp = config.sim.mode.controllerConfig.payloadCompensation;

if payloadComp == ControllerPayloadCompensation.NotActive
    Symb_f_jacobian = A_eq_WP(x, u);
elseif payloadComp == ControllerPayloadCompensation.Active
    Symb_f_jacobian = A_eq_P_obsv(x, u);
else
    error("Invalid Payload Compensation value");
end
