classdef ExtDisturbances < Simulink.IntEnumType
    enumeration
        None(1)
        ConstForce(2)
        Wavy(3)
        GaussianForce(4)
        GaussianForceAndWavy(5)
    end
end