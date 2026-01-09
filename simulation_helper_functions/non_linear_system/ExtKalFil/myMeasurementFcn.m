function yk = myMeasurementFcn(xk)
% myMeasurementFcn
% Measurement model: y(k) = h(x(k))

q = xk(1:2);

% Encoders measure joint angles
yk = q;
end