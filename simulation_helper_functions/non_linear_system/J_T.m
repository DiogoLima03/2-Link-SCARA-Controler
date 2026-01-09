function output = J_T(q, config)
% J_T Geometrical Jacobian transpose

    q1 = q(1);
    q2 = q(2);

    l1 = config.robot.geom.l1;
    l2 = config.robot.geom.l2;

    output = [ ...
        -12*cos(q1 + q2) - l1*cos(q1), -l2*sin(q1 + q2) - l1*sin(q1); ...
        -l2*cos(q1 + q2),              -l2*sin(q1 + q2) ...
    ];
end
