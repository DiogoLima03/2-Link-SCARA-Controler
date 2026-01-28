function output = D_mat_plant(config)
% D_mat Joint viscous damping matrix D
% % used to simulate the plant, not the controller
    D1 = config.robot_plant.dyn.D1;
    D2 = config.robot_plant.dyn.D2;

    output = diag([D1, D2]);
end
