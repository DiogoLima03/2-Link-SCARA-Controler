function output = c_vec(q, dot_q, config)
%c_vec Coriolis/centrifugal vector c(q,qdot)

    % States
    q2  = q(2);
    dq1 = dot_q(1);
    dq2 = dot_q(2);

    % Parameters
    l1  = config.robot.geom.l1;
    lc2 = config.robot.geom.lc2;
    m2  = config.robot.dyn.m2;

    % c(q,qdot) vector
    output = [ ...
        -dq2*(2*dq1 + dq2)*l1*lc2*m2*sin(q2); ...
         dq1^2*l1*lc2*m2*sin(q2) ...
    ];
end
