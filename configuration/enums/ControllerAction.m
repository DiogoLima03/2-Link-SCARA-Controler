classdef ControllerAction < Simulink.IntEnumType
    enumeration
        PD_zeta_omega(1)
        PD_LQR(2)
        PID_zeta_omega(3)
        PID_LQR(4)
    end
end