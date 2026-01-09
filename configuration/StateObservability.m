classdef StateObservability < Simulink.IntEnumType
    enumeration
        FullStateMeasurement(1)
        StateReconstructionDevLowPassFilt(2)
        StateReconstructionDirtyDev(3)
        ExtendedKalmanBucyFilter(4)
        UnscentedKalmanFilter(5)

        % not implemented
        LuenbergerObserver(6) % for: Linear system + (no modelling noise) & (no measurement noise)
        
        % For noisy systems
        FullStateMeasurementWithLowPassFilter(7) % for noisy systems
        StateReconstructionWithLowPassFilter(8) % for noisy systems
        BucyKalmanFilterFiniteTime(9) % for LINEAR noisy systems
        BucyKalmanFilterInfiniteTime(10) % for LINEAR noisy systems
        
         % for NON-LINEAR noisy systems
        HighGainObserver(11) % for: Non-Linear system + (no modelling noise) & (no measurement noise)
        ExtendedHighGainObserver(12) % for: Linear system + (WIDTH modelling noise) & (measurement noise - possible with some modifications)
    end
end