classdef MeasDisturbances < Simulink.IntEnumType
    enumeration
        None(1)
        ConstDisturbance(2)
        GaussianDisturbance(3)
    end
end