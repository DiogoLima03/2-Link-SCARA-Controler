function output = foward_kin(q, config)
% p_e End-effector position 

    q1 = q(1);
    q2 = q(2);

    l1 = config.robot.geom.l1;
    l2 = config.robot.geom.l2;

    output = [ ...
        -l1*sin(q1) - l2*sin(q1 + q2); ...
         l1*cos(q1) + l2*cos(q1 + q2) ...
    ];
end
