clear; close all; clc;

%% Importing paramaters from file ('./../configuration/sim_config_parameters.m')

% Adding the matlab RaM-ART folders to matlab path

addpath(fullfile(pwd,'./../configuration'));


% Adding configuration parameters to the workspace 

run('./../configuration/sim_config_parameters.m')

%% Variables robot (from config struct)

% --- Link geometry ---
l1_v  = config.robot.geom.l1;
l2_v  = config.robot.geom.l2;
lc1_v = config.robot.geom.lc1;
lc2_v = config.robot.geom.lc2;

% --- Cross-section geometry ---
b_outer = config.robot.cs.b_outer;
h       = config.robot.cs.h;

t   = config.robot.cs.t;   % keep original name if other code expects it
t_b = t;                   % RectangleHollowLink expects separate thicknesses
t_h = t;

L   = config.robot.cs.L;
rho = config.robot.cs.rho;

% --- Mass and inertia ---
% Option A (recommended): use the already computed values from config
m1_v = config.robot.dyn.m1;
m2_v = config.robot.dyn.m2;
I1_v = config.robot.dyn.I1;
I2_v = config.robot.dyn.I2;

% --- Joint damping ---
D1_v = config.robot.dyn.D1;
D2_v = config.robot.dyn.D2;

% --- External payload ---
m_ext_v = config.robot.payload.mass;

% --- Gravity configuration ---
alpha_v = config.robot.gravity.alpha;


%% Symbolic equations
% symbolic variables
syms I1 I2 m1 m2 l1 l2 lc1 lc2 D1 D2 alpha real
syms q1 q2 q1dot q2dot u1 u2 real

g = 9.81; % m/s^2

% Mass matrix M(q)
M = [ I1 + I2 + l1^2*m2 + lc1^2*m1 + lc2^2*m2 + 2*l1*lc2*m2*cos(q2), ...
      I2 + lc2^2*m2 + l1*lc2*m2*cos(q2);
      I2 + lc2^2*m2 + l1*lc2*m2*cos(q2), ...
      I2 + lc2^2*m2 ];

% Coriolis/centrifugal vector c(q,qdot)
c = [ -q2dot*(2*q1dot + q2dot)*l1*lc2*m2*sin(q2);
       q1dot^2*l1*lc2*m2*sin(q2) ];

D = diag([D1, D2]);

% Gravity vector G(q)
G = [ g*(l1*m2 + lc1*m1)*sin(alpha - q1) - g*lc2*m2*sin(q1 - alpha + q2);
     -g*lc2*m2*sin(q1 - alpha + q2) ];

% End-effector position p_e(q)
pe = [ -l1*sin(q1) - l2*sin(q1 + q2);
        l1*cos(q1) + l2*cos(q1 + q2) ];

% Input mapping S*u
S  = [1 0;
      0 1];
u  = [u1; u2];

Su = S*u;

% Formulation of the external forces to the system

% Jacobian to the end-effector
J = jacobian(pe, [q1 q2]);
J_T = simplify(J.');
J_inv = inv(J);

% External force and just mass suspense ate the end-effector
syms m_ext

W = [0; -m_ext*g];

%% Non-Linear State Space Form

syms q1 q2 w1 w2

% states
x = [q1;
    q2;
    w1;
    w2];

q = [q1;
    q2];

w = [w1;
    w2];

% rewrite the term that were expressed using the old state name "q1dot"-> "w1" and "q2dot"->"w2" 
c = subs(c, [q1dot, q2dot], [w1, w2]);   % c expressed in terms of w

%% Non-Linear State Space Form (with payload)
NLSSF = struct(); 

NLSSF.f = [w;
           M\(-c - D*w - G + S*u + J_T*W)];

NLSSF.h = q;

%% Non-Linear State Space Form Without Payload
NLSSFWP = struct();

NLSSFWP.f = [w;
             M\(-c - D*w - G + S*u)];

NLSSFWP.h = q;
%% Linear state-space Form (LSSF)

sec_LSSF_display_terminal = true;


LSSFWP.A = simplify(jacobian(NLSSFWP.f, x));
LSSFWP.B = simplify(jacobian(NLSSFWP.f, u));
LSSFWP.C = simplify(jacobian(NLSSFWP.h, x));
LSSFWP.D = simplify(jacobian(NLSSFWP.h, u));

sym_vec = [q1, q2, w1, w2, ...
           l1, l2, lc1, lc2, ...
           m1, m2, I1, I2, ...
           D1, D2, alpha];

param_vec = [l1_v, l2_v, lc1_v, lc2_v, ...
             m1_v, m2_v, I1_v, I2_v, ...
             D1_v, D2_v, alpha_v];


% Equilibrium Point: (q1, q2) = (0, 0), w1 = w2 = 0

EqPoint_00 = struct();

vals_00 = [ 0,  0,  0,  0, param_vec];  % [q1 q2 w1 w2 ...params]

EqPoint_00.A = double(subs(LSSFWP.A, sym_vec, vals_00));
EqPoint_00.B = double(subs(LSSFWP.B, sym_vec, vals_00));
EqPoint_00.C = double(subs(LSSFWP.C, sym_vec, vals_00));
EqPoint_00.D = double(subs(LSSFWP.D, sym_vec, vals_00));

if sec_LSSF_display_terminal
    disp("Equilibrium Point: (q1, q2) = (0, 0), w1 = w2 = 0");
    disp("A:");
    disp(EqPoint_00.A);
    disp("B:");
    disp(EqPoint_00.B);
    disp("C:");
    disp(EqPoint_00.C);
    disp("D:");
    disp(EqPoint_00.D);
end

% Equilibrium Point: (q1, q2) = (pi, 0), w1 = w2 = 0

EqPoint_pi0 = struct();

vals_pi0 = [ pi,  0,  0,  0, param_vec];

EqPoint_pi0.A = double(subs(LSSFWP.A, sym_vec, vals_pi0));
EqPoint_pi0.B = double(subs(LSSFWP.B, sym_vec, vals_pi0));
EqPoint_pi0.C = double(subs(LSSFWP.C, sym_vec, vals_pi0));
EqPoint_pi0.D = double(subs(LSSFWP.D, sym_vec, vals_pi0));

if sec_LSSF_display_terminal
    disp("Equilibrium Point: (q1, q2) = (pi, 0), w1 = w2 = 0");
    disp("A:");
    disp(EqPoint_pi0.A);
    disp("B:");
    disp(EqPoint_pi0.B);
    disp("C:");
    disp(EqPoint_pi0.C);
    disp("D:");
    disp(EqPoint_pi0.D);
end

% Equilibrium Point: (q1, q2) = (0, pi), w1 = w2 = 0

EqPoint_0pi = struct();

vals_0pi = [ 0,  pi,  0,  0, param_vec];

EqPoint_0pi.A = double(subs(LSSFWP.A, sym_vec, vals_0pi));
EqPoint_0pi.B = double(subs(LSSFWP.B, sym_vec, vals_0pi));
EqPoint_0pi.C = double(subs(LSSFWP.C, sym_vec, vals_0pi));
EqPoint_0pi.D = double(subs(LSSFWP.D, sym_vec, vals_0pi));

if sec_LSSF_display_terminal
    disp("Equilibrium Point: (q1, q2) = (0, pi), w1 = w2 = 0");
    disp("A:");
    disp(EqPoint_0pi.A);
    disp("B:");
    disp(EqPoint_0pi.B);
    disp("C:");
    disp(EqPoint_0pi.C);
    disp("D:");
    disp(EqPoint_0pi.D);
end

% Equilibrium Point: (q1, q2) = (pi, pi), w1 = w2 = 0

EqPoint_pipi = struct();

vals_pipi = [ pi,  pi,  0,  0, param_vec];

EqPoint_pipi.A = double(subs(LSSFWP.A, sym_vec, vals_pipi));
EqPoint_pipi.B = double(subs(LSSFWP.B, sym_vec, vals_pipi));
EqPoint_pipi.C = double(subs(LSSFWP.C, sym_vec, vals_pipi));
EqPoint_pipi.D = double(subs(LSSFWP.D, sym_vec, vals_pipi));

if sec_LSSF_display_terminal
    disp("Equilibrium Point: (q1, q2) = (pi, pi), w1 = w2 = 0");
    disp("A:");
    disp(EqPoint_pipi.A);
    disp("B:");
    disp(EqPoint_pipi.B);
    disp("C:");
    disp(EqPoint_pipi.C);
    disp("D:");
    disp(EqPoint_pipi.D);
end


%% Stability (S) - Eigenvalues of Linearised Systems at Each Equilibrium

sec_S_display_terminal = true;

% Eigenvalues calculation
EqPoint_00.eigA   = eig(EqPoint_00.A);
EqPoint_pi0.eigA  = eig(EqPoint_pi0.A);
EqPoint_0pi.eigA  = eig(EqPoint_0pi.A);
EqPoint_pipi.eigA = eig(EqPoint_pipi.A);

if sec_S_display_terminal
    disp('Eigenvalues of A at equilibrium (q1, q2) = (0, 0):');
    disp(EqPoint_00.eigA.');
    
    disp('Eigenvalues of A at equilibrium (q1, q2) = (\pi, 0):');
    disp(EqPoint_pi0.eigA.');
    
    disp('Eigenvalues of A at equilibrium (q1, q2) = (0, \pi):');
    disp(EqPoint_0pi.eigA.');
    
    disp('Eigenvalues of A at equilibrium (q1, q2) = (\pi, \pi):');
    disp(EqPoint_pipi.eigA.');
end

%% Phase Portrait on plane q1 q2

M_fun = matlabFunction(M, 'Vars', {q1, q2, l1, l2, lc1, lc2, m1, m2, I1, I2});
c_fun = matlabFunction(c, 'Vars', {q1, q2, w1, w2, l1, l2, lc1, lc2, m1, m2});
G_fun = matlabFunction(G, 'Vars', {q1, q2, l1, l2, lc1, lc2, m1, m2, alpha});

params = struct('l1_v', l1_v, 'l2_v', l2_v, ...
                'lc1_v', lc1_v, 'lc2_v', lc2_v, ...
                'm1_v', m1_v, 'm2_v', m2_v, ...
                'I1_v', I1_v, 'I2_v', I2_v, ...
                'D1_v', D1_v, 'D2_v', D2_v, ...
                'alpha_v', alpha_v, ...
                'M_fun', M_fun, ...
                'c_fun', c_fun, ...
                'G_fun', G_fun);


function dx = scara_dynamics(t, x, params)
    
    % x = [q1; q2; w1; w2]
    q1 = x(1);  q2 = x(2);
    w1 = x(3);  w2 = x(4);

    % unpack parameters
    l1   = params.l1_v;
    l2   = params.l2_v;
    lc1  = params.lc1_v;
    lc2  = params.lc2_v;
    m1   = params.m1_v;
    m2   = params.m2_v;
    I1   = params.I1_v;
    I2   = params.I2_v;
    D1   = params.D1_v;
    D2   = params.D2_v;
    alpha = params.alpha_v;

    % get function handles from params
    M_fun = params.M_fun;
    c_fun = params.c_fun;
    G_fun = params.G_fun;

    % build matrices
    Mq = M_fun(q1, q2, l1, l2, lc1, lc2, m1, m2, I1, I2);
    cq = c_fun(q1, q2, w1, w2, l1, l2, lc1, lc2, m1, m2);
    gq = G_fun(q1, q2, l1, l2, lc1, lc2, m1, m2, alpha);

    D = diag([D1, D2]);
    u = [0; 0];  % no input

    dq   = [w1; w2];
    dw   = Mq \ (-cq - D*[w1; w2] - gq + u);

    dx = [dq; dw];
end


tspan = [0 10];

% choose some initial positions (with zero velocities)
% Generate many initial conditions on a grid in q1–q2 space
q1_vals = linspace(-pi, pi, 8);     
q2_vals = linspace(-pi/100, pi/100, 8);

initial_qs = [];
for i = 1:length(q1_vals)
    for j = 1:length(q2_vals)
        initial_qs = [initial_qs; q1_vals(i), q2_vals(j)];
    end
end

figure; hold on;

for k = 1:size(initial_qs,1)
    q10 = initial_qs(k,1);
    q20 = initial_qs(k,2);
    x0  = [q10; q20; 0; 0];   % w1 = w2 = 0

    [t, x] = ode45(@(t,x) scara_dynamics(t,x,params), tspan, x0);

    % Plot full trajectory
    plot(x(:,1), x(:,2), "k", 'LineWidth', 0.25);

        % Add arrows at intervals along trajectory
    stride = 50; % modify for more/less arrows
    arrow_len = 0.5;  % fixed visible length in q1-q2 plane

    for idx = 1:stride:length(x)-1
        q1_head = x(idx,1);
        q2_head = x(idx,2);

        dq1 = x(idx+1,1) - x(idx,1);
        dq2 = x(idx+1,2) - x(idx,2);

        dir_vec = [dq1 dq2];
        n = norm(dir_vec);

        if n > 0
            dir_vec = dir_vec / n * arrow_len;  % normalize and scale
            quiver(q1_head, q2_head, dir_vec(1), dir_vec(2), ...
                'Color', [0, 1, 0.314], ...
                'MaxHeadSize', 35, ...
                'AutoScale', 'off');  % don't rescale again
        end
    end
end

xlabel('q_1 [rad]');
ylabel('q_2 [rad]');
title('Projected phase portrait in the (q_1, q_2)-plane with arrows');
grid on;
% Define equilibrium ranges
q1_eq_vals = pi * (-1:1); % covers [-6,6] and more
q2_eq_vals = pi * (-4:4); % covers [-15,15] and more

[q1_eq_grid, q2_eq_grid] = meshgrid(q1_eq_vals, q2_eq_vals);

eq_points = [q1_eq_grid(:), q2_eq_grid(:)];

% Plot equilibrium points
scatter(eq_points(:,1), eq_points(:,2), ...
        60, 'r', 'filled', 'MarkerEdgeColor', 'k');

% Label them
for i = 1:size(eq_points,1)
    text(eq_points(i,1) + 0.1, eq_points(i,2) + 0.1, ...
        sprintf('(%.0fπ, %.0fπ)', eq_points(i,1)/pi, eq_points(i,2)/pi), ...
        'FontSize', 6, 'Color', 'k');
end

hold off;



%% Simulated Phase Portrait - Potential Energy

g = 9.81;

% Expanded range for q1 and q2
q1 = linspace(-13/6*pi, pi/6, 20);   % expanded domain
q2 = linspace(-5/2*pi, pi/2, 20);
[Q1, Q2] = meshgrid(q1, q2);

% Potential energy
H = g*(l1_v*m2_v + lc1_v*m1_v).*cos(Q1) + ...
    g*lc2_v*m2_v.*cos(Q1 + Q2);

% Compute gradient numerically
dq1 = q1(2) - q1(1);
dq2 = q2(2) - q2(1);
[dH_dq1, dH_dq2] = gradient(H, dq1, dq2);

% Overdamped "gradient descent" vector field
U = -dH_dq1;
V = -dH_dq2;

% Plot energy contours
figure; hold on;
contourf(Q1, Q2, H, 40, 'LineColor', 'none');
colormap(turbo);
colorbar;
xlabel('q_1 [rad]');
ylabel('q_2 [rad]');
title('Expanded Potential Energy Map with Streamlines');
axis equal tight;

% Streamline seeding grid (larger range → more visible trajectories)
[sx, sy] = meshgrid(linspace(-2*pi, 4*pi, 20), ...
                    linspace(-2*pi, 4*pi, 20));

% Compute streamlines using stream2 (better than streamline)
streams = stream2(Q1, Q2, U, V, sx, sy);

% Plot the streamlines
for k = 1:length(streams)
    sl = streams{k};
    if ~isempty(sl)
        plot(sl(:,1), sl(:,2), 'Color', "k", 'LineWidth', 1);
    end
end

hold off;




%% Potential Energy

% Define q1 and q2 ranges
q1 = (-2*pi):0.05:(2*pi);
q2 = (-2*pi):0.05:(2*pi);

% Create meshgrid
[Q1, Q2] = meshgrid(q1, q2);

H_eq_pi_0 = g*(l1_v*m2_v + lc1_v*m1_v).*cos(pi) ...
    + g*lc2_v*m2_v.*cos(pi + 0);

q_min = [pi,0];

q_eq_study = [0, pi];
   
delta_q = q_min - q_eq_study;

% Potential energy function
H = g*(l1_v*m2_v + lc1_v*m1_v).*cos(Q1 + delta_q(1)) ...
    + g*lc2_v*m2_v.*cos(Q1 + delta_q(1) + Q2 + delta_q(2)) - H_eq_pi_0;

% Plot
figure;
hold on;
surf(q1, q2, H);
shading interp;
xlabel('q_1 [rad]');
ylabel('q_2 [rad]');
zlabel('E_{pot} [J]');
title('Potential Energy Surface of the 2-Link SCARA Robot');
colorbar;
view(3);
grid on;
    
z_line = linspace(min(H(:)), max(H(:)), 100);

q1_line = q_eq_study(1) * ones(size(z_line));   % Correct
q2_line = q_eq_study(2) * ones(size(z_line));   % Correct

plot3(q1_line, q2_line, z_line, 'k', 'LineWidth', 3);
hold off;

%% Controlability - Linear System

Co = ctrb(EqPoint_pi0.A, EqPoint_pi0.B);
rCo = rank(Co);

n = rank(EqPoint_pi0.A);
isCtrl = (rCo == n);

if isCtrl
    isCtrl_ = "True";
else
    isCtrl_ = "False";
end

fprintf("rank(Ctrb) = %d (n = %d), controllable (%s)\n", rCo, n, isCtrl_);

%% Observability - Linear System

Ob = obsv(EqPoint_pi0.A, EqPoint_pi0.C);
rOb = rank(Ob);

n = rank(EqPoint_pi0.A);
isObsv = (rOb == n);

if isObsv
    isObsv_ = "True";
else
    isObsv_ = "False";
end

fprintf("rank(Obsv) = %d (n = %d), observable (%s)\n", rCo, n, isObsv_);

%% EKF F Jacobian matrix



% Equilibrium Point undefined: x = [q1; q2; dot_q1; dot_q2]

syms q1 q2 w1 w2

sym_vec = [l1, l2, lc1, lc2, ...
           m1, m2, I1, I2, ...
           D1, D2, alpha];

param_vec = [l1_v, l2_v, lc1_v, lc2_v, ...
             m1_v, m2_v, I1_v, I2_v, ...
             D1_v, D2_v, alpha_v];

Symb_f_jacobian = simplify(subs(LSSFWP.A, sym_vec, param_vec));

%f_jacobian = matlabFunction(Symb_f_jacobian, 'File','f_jacobian_sym.m', 'Vars', {q1, q2, w1, w2, u1, u2});


%% LQR design (discreat)

Ts = config.sim.var.time_step;

n = 2;
A = [zeros(n) eye(n);
     zeros(n) zeros(n)];
B = [zeros(n);
     eye(n)];

sysc = ss(A,B,eye(2*n),zeros(2*n,n));
sysd = c2d(sysc, Ts, 'zoh');

Ad = sysd.A;
Bd = sysd.B;

% Gains
Q = diag([100 100 1 50]);
R = diag([1 1]);

K = dlqr(Ad, Bd, Q, R);

Kp = -K(:,1:2)
Kd = -K(:,3:4)
