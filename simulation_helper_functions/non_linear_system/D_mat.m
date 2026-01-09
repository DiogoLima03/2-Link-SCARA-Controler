function output = D_mat(config)
%D_mat Joint viscous damping matrix D

    D1 = config.robot.dyn.D1;
    D2 = config.robot.dyn.D2;

    output = diag([D1, D2]);
end
