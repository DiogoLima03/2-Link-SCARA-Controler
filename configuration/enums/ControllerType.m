classdef ControllerType < Simulink.IntEnumType
    enumeration
        None(1)
        LinearSS(2)      % Linear State Feedback 
        NonLinearFL(3)   % Non Linear - Feedback Linearisation 
    end
end