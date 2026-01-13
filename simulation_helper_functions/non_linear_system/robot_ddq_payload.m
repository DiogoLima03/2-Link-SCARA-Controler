function [ddot_q] = robot_ddq_payload(q, dot_q, tau, W, config)
% Non-Linear State Space Function

M_matrix = M(q, config);
tau_total = tau + J_T(q, config)*W - c_vec(q, dot_q, config) - G_vec(q, config) - D_mat(config)*dot_q;

ddot_q = M_matrix\(tau_total);

end