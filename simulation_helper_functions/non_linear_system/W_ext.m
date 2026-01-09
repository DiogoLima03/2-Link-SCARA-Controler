function output = W_ext(config)
% W_ext External force applied to th ened-effector

    g = 9.81;
    m_load = config.robot.payload.mass;

    output = [0;
              -m_load*g];
end
