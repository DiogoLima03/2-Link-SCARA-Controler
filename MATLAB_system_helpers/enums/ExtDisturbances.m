classdef ExtDisturbances < Simulink.IntEnumType
    enumeration
        None(1)
        ConstForce(2)
        GaussianForce(3)
        GaussianAndConstForce(4)
    end
end