function out = J_inv(q, config)
    q1 = q(1);
    q2 = q(2);
    
    l1 = config.robot.geom.l1;
    l2 = config.robot.geom.l2;

    out = [ sin(q1 + q2)/(l1*cos(q1 + q2)*sin(q1) - l1*sin(q1 + q2)*cos(q1)),                         -cos(q1 + q2)/(l1*cos(q1 + q2)*sin(q1) - l1*sin(q1 + q2)*cos(q1)); 
            -(l2*sin(q1 + q2) + l1*sin(q1))/(l1*l2*cos(q1 + q2)*sin(q1) - l1*l2*sin(q1 + q2)*cos(q1)), (l2*cos(q1 + q2) + l1*cos(q1))/(l1*l2*cos(q1 + q2)*sin(q1) - l1*l2*sin(q1 + q2)*cos(q1))];
end