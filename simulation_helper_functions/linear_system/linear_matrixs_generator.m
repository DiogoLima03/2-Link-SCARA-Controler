
linMatGen(config);

function [] = linMatGen(config) 
    % Function objective: Generate functions for linearised system matrices A, B, C, D
    % using a input x_eq (the desired equilibrium point for the linearisation)
    
    % Variables robot (from config struct)
    
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
    
    
    % Symbolic equations
    % symbolic variables
    syms I1 I2 m1 m2 l1 l2 lc1 lc2 D1 D2 real
    alpha = sym('alpha', 'real');
    syms q1 q2 dot_q1 dot_q2 u1 u2 real
    
    g = 9.81; % m/s^2
    
    % Mass matrix M(q)
    M = [ I1 + I2 + l1^2*m2 + lc1^2*m1 + lc2^2*m2 + 2*l1*lc2*m2*cos(q2), ...
          I2 + lc2^2*m2 + l1*lc2*m2*cos(q2);
          I2 + lc2^2*m2 + l1*lc2*m2*cos(q2), ...
          I2 + lc2^2*m2 ];
    
    % Coriolis/centrifugal vector c(q,qdot)
    c = [ -dot_q2*(2*dot_q1 + dot_q2)*l1*lc2*m2*sin(q2);
           dot_q1^2*l1*lc2*m2*sin(q2) ];
    
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
    
    % External force and just mass suspense ate the end-effector
    syms m_ext
    
    W = [0; -m_ext*g];
    
    % States
    
    % states
    x = [q1;
         q2;
         dot_q1;
         dot_q2];
    
    q = [q1;
         q2];
    
    dot_q = [dot_q1;
             dot_q2];
    
    % Non-Linear State Space Form (with payload)
    NLSSF = struct(); 
    
    NLSSF.f = [dot_q;
               M\(-c - D*dot_q - G + S*u + J_T*W)];
    
    NLSSF.h = q;
    
    % Non-Linear State Space Form Without Payload
    NLSSFWP = struct();
    
    NLSSFWP.f = [dot_q;
                 M\(-c - D*dot_q - G + S*u)];
    
    NLSSFWP.h = q;
    
    % Linear state-space Form Without Payload (LSSF)
    
    
    LSSF.A = simplify(jacobian(NLSSF.f, x));
    LSSF.B = simplify(jacobian(NLSSF.f, u));
    LSSF.C = simplify(jacobian(NLSSF.h, x));
    LSSF.D = simplify(jacobian(NLSSF.h, u));
    
    sym_vec = [l1, l2, lc1, lc2, ...
               m1, m2, I1, I2, ...
               D1, D2, alpha, m_ext];
    
    param_vec = [l1_v, l2_v, lc1_v, lc2_v, ...
                 m1_v, m2_v, I1_v, I2_v, ...
                 D1_v, D2_v, alpha_v, m_ext_v];
    
    Symb_A_P = simplify(subs(LSSF.A, sym_vec, param_vec));
    Symb_B_P = simplify(subs(LSSF.B, sym_vec, param_vec));
    Symb_C_P = simplify(subs(LSSF.C, sym_vec, param_vec));
    Symb_D_P = simplify(subs(LSSF.D, sym_vec, param_vec));
    
    A_eq_P = matlabFunction(Symb_A_P, 'File','A_eq_P.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});
    B_eq_P = matlabFunction(Symb_B_P, 'File','B_eq_P.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});
    C_eq_P = matlabFunction(Symb_C_P, 'File','C_eq_P.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});
    D_eq_P = matlabFunction(Symb_D_P, 'File','D_eq_P.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});

    % Linear state-space Form Without Payload (LSSFWP)
    
    LSSFWP.A = simplify(jacobian(NLSSFWP.f, x));
    LSSFWP.B = simplify(jacobian(NLSSFWP.f, u));
    LSSFWP.C = simplify(jacobian(NLSSFWP.h, x));
    LSSFWP.D = simplify(jacobian(NLSSFWP.h, u));
    
    sym_vec = [l1, l2, lc1, lc2, ...
               m1, m2, I1, I2, ...
               D1, D2, alpha];
    
    param_vec = [l1_v, l2_v, lc1_v, lc2_v, ...
                 m1_v, m2_v, I1_v, I2_v, ...
                 D1_v, D2_v, alpha_v];
    
    Symb_A_WP = simplify(subs(LSSFWP.A, sym_vec, param_vec));
    Symb_B_WP = simplify(subs(LSSFWP.B, sym_vec, param_vec));
    Symb_C_WP = simplify(subs(LSSFWP.C, sym_vec, param_vec));
    Symb_D_WP = simplify(subs(LSSFWP.D, sym_vec, param_vec));
    
    A_eq_WP = matlabFunction(Symb_A_WP, 'File','A_eq_WP.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});
    B_eq_WP = matlabFunction(Symb_B_WP, 'File','B_eq_WP.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});
    C_eq_WP = matlabFunction(Symb_C_WP, 'File','C_eq_WP.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});
    D_eq_WP = matlabFunction(Symb_D_WP, 'File','D_eq_WP.m', 'Vars', {[q1, q2, dot_q1, dot_q2, u1, u2]});

end