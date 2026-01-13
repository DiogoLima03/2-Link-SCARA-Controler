function [ddot_q] = robot_ddq(q, dot_q, tau, config)
% Non-Linear State Space Function

M_matrix = M(q, config);
tau_total = tau - c_vec(q, dot_q, config) - G_vec(q, config) - D_mat(config)*dot_q;

ddot_q = M_matrix\(tau_total);

end