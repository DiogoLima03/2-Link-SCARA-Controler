function F = myStateTransitionJacobianFcn(xk, uk)
%#codegen
% Jacobian of RK4 discrete-time transition x_{k+1} = Phi(x_k,u_k)

f_jacobian = @(x,u) f_jacobian_sym(x(1), x(2), x(3), x(4), u(1), u(2));

config = localConfig();
Ts = config.sim.var.time_step;

I = eye(numel(xk));

% --- Stage 1 ---
x1 = xk;
k1 = f(x1, uk, config);
A1 = f_jacobian(x1, uk);               % df/dx at x1
dk1 = A1;                              % dk1/dx

% --- Stage 2 ---
x2 = xk + 0.5*Ts*k1;
k2 = f(x2, uk, config);
A2 = f_jacobian(x2, uk);
dx2 = I + 0.5*Ts*dk1;                  % d(x + 0.5Ts k1)/dx
dk2 = A2 * dx2;                        % dk2/dx

% --- Stage 3 ---
x3 = xk + 0.5*Ts*k2;
k3 = f(x3, uk, config);
A3 = f_jacobian(x3, uk);
dx3 = I + 0.5*Ts*dk2;
dk3 = A3 * dx3;

% --- Stage 4 ---
x4 = xk + Ts*k3;
k4 = f(x4, uk, config);
A4 = f_jacobian(x4, uk);
dx4 = I + Ts*dk3;
dk4 = A4 * dx4;

% --- RK4 discrete Jacobian ---
F = I + (Ts/6)*(dk1 + 2*dk2 + 2*dk3 + dk4);
end
