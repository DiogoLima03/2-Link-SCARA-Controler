function dot_x = f(x, u, config)
    
% states
q = x(1:2);
dot_q = x(3:4);


% input torque
tau = u;

% the state space formulation, dot_q = f(x,u):
dot_x = [dot_q;
         robot_ddq(q, dot_q, tau, config)];

end
