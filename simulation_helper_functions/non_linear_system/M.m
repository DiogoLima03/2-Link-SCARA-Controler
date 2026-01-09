function output = M(q, config)
%M Inertia matrix M(q) for 2-DOF planar arm 

    % States
    q2 = q(2);
    
    % Parameters
    l1  = config.robot.geom.l1;
    lc1 = config.robot.geom.lc1;
    lc2 = config.robot.geom.lc2;

    m1 = config.robot.dyn.m1;
    m2 = config.robot.dyn.m2;

    I1 = config.robot.dyn.I1;
    I2 = config.robot.dyn.I2;

    % Inertia matrix 
    output = [ ...
        I1 + I2 + l1^2*m2 + lc1^2*m1 + lc2^2*m2 + 2*l1*lc2*m2*cos(q2),   I2 + lc2^2*m2 + l1*lc2*m2*cos(q2); ...
        I2 + lc2^2*m2 + l1*lc2*m2*cos(q2),                              I2 + lc2^2*m2 ...
    ];
end
