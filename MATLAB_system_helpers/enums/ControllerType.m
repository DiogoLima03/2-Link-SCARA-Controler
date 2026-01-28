classdef ControllerType < Simulink.IntEnumType
    enumeration
        None(1)
        LinearSS(2)      % Linear State Feedback 
        PDwithGravity(3) % PD control with gravity compensation
        NonLinearFL(4)   % Non Linear - Feedback Linearisation 
    end
end