function k = linear_controller_LQR(x, u, Q, R, config)

    A = A_eq_WP(x,u);
    B = B_eq_WP(x,u);
    C = C_eq_WP(x,u);
    D = D_eq_WP(x,u);
    
    ss_cont = ss(A, B, C, D);

    Ts = config.sim.var.time_step;
    ss_disc = c2d(ss_cont, Ts, 'zoh');

    k = dlqr(ss_disc.A, ss_disc.B, Q, R);
end