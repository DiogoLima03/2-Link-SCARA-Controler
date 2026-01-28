function output = G_vec_plant(q, config)
% G_vec Gravity vector G(q) 
% used to simulate the plant, not the controller

    % States
    q1 = q(1);
    q2 = q(2);

    % Parameters
    l1   = config.robot_plant.geom.l1;
    lc1  = config.robot_plant.geom.lc1;
    lc2  = config.robot_plant.geom.lc2;

    m1 = config.robot_plant.dyn.m1;
    m2 = config.robot_plant.dyn.m2;

    alpha = config.robot_plant.gravity.alpha;

    % Gravity constant
    g = 9.81;

    % G(q) vector 
    output = [ ...
        g*(l1*m2 + lc1*m1)*sin(alpha - q1) - g*lc2*m2*sin(q1 - alpha + q2); ...
       -g*lc2*m2*sin(q1 - alpha + q2) ...
    ];
end
