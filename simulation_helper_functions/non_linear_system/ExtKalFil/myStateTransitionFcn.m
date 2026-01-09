function xk1 = myStateTransitionFcn(xk, uk)
%#codegen
% Discrete-time state transition using RK4

% ---- Parameters ----
config = localConfig(); 
Ts = config.sim.var.time_step;

% ---- RK4 integration ----
k1 = f(xk,                uk, config);
k2 = f(xk + 0.5*Ts*k1,     uk, config);
k3 = f(xk + 0.5*Ts*k2,     uk, config);
k4 = f(xk + Ts*k3,         uk, config);

xk1 = xk + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function config = localConfig()
% robot params
config.robot.geom.l1  = 0.6;
config.robot.geom.l2  = 0.6;
config.robot.geom.lc1 = 0.3;
config.robot.geom.lc2 = 0.3;

config.robot.dyn.m1 = 2.6136;
config.robot.dyn.m2 = 2.6136;
config.robot.dyn.I1 = 0.07973658;
config.robot.dyn.I2 = 0.07973658;
config.robot.dyn.D1 = 0.1;
config.robot.dyn.D2 = 0.1;

config.robot.payload.mass = 0.4;
config.robot.gravity.alpha = 0;

% simulation step
config.sim.var.time_step = 0.004;

end
