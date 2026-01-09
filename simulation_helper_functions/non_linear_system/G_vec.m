function output = G_vec(q, config)
%G_vec Gravity vector G(q) 

    % States
    q1 = q(1);
    q2 = q(2);

    % Parameters
    l1   = config.robot.geom.l1;
    lc1  = config.robot.geom.lc1;
    lc2  = config.robot.geom.lc2;

    m1 = config.robot.dyn.m1;
    m2 = config.robot.dyn.m2;

    alpha = config.robot.gravity.alpha;

    % Gravity constant
    g = 9.81;

    % G(q) vector 
    output = [ ...
        g*(l1*m2 + lc1*m1)*sin(alpha - q1) - g*lc2*m2*sin(q1 - alpha + q2); ...
       -g*lc2*m2*sin(q1 - alpha + q2) ...
    ];
end
