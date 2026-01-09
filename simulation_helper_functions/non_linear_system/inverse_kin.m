function [q, dq, ddq, isReachable, q_all] = inverse_kin(p, v, a, config, elbow)
% Inverse kinematics + differential kinematics for planar 2-link arm (SCARA-style)
%
% Your FK:
%   x = -l1*sin(q1) - l2*sin(q1+q2)
%   y =  l1*cos(q1) + l2*cos(q1+q2)
%
% Inputs:
%   p     : [2x1] end-effector position [x; y] [m]
%   v     : [2x1] end-effector velocity [dx; dy] [m/s]
%   a     : [2x1] end-effector acceleration [ddx; ddy] [m/s^2]
%   config: struct with config.robot.geom.l1, l2
%   elbow : "up" or "down" (default: "up")
%
% Outputs:
%   q           : [2x1] joint angles [q1; q2] [rad]
%   dq          : [2x1] joint velocities [rad/s]
%   ddq         : [2x1] joint accelerations [rad/s^2]
%   isReachable : logical
%   q_all       : [2x2] [q_up q_down]

    if nargin < 5 || isempty(elbow)
        elbow = "up";
    end

    % Ensure column vectors
    p = p(:); 
    x = p(1); y = p(2);

    % velocity is optional
    hasVel = (nargin >= 2) && ~isempty(v);
    if hasVel
        v = v(:);
    else
        v = zeros(2,1);
    end

    % acceleration is optional
    hasAcc = (nargin >= 3) && ~isempty(a);
    if hasAcc
        a = a(:);
    end

    l1 = config.robot.geom.l1;
    l2 = config.robot.geom.l2;

    % Map to standard 2R coordinates (Xs,Ys)
    % X_std = y, Y_std = -x
    Xs = y;
    Ys = -x;

    % Reachability using cosine law
    r2 = Xs^2 + Ys^2;
    c2 = (r2 - l1^2 - l2^2) / (2*l1*l2);

    tol = 1e-12;
    isReachable = (c2 >= -1 - tol) && (c2 <= 1 + tol);
    if ~isReachable
        c2 = min(1, max(-1, c2));
    end

    s2_pos = sqrt(max(0, 1 - c2^2));
    s2_neg = -s2_pos;

    q2_up   = atan2(s2_pos, c2);
    q2_down = atan2(s2_neg, c2);

    q1_up   = atan2(Ys, Xs) - atan2(l2*s2_pos, l1 + l2*c2);
    q1_down = atan2(Ys, Xs) - atan2(l2*s2_neg, l1 + l2*c2);

    q_up   = [wrapToPiLocal(q1_up);   wrapToPiLocal(q2_up)];
    q_down = [wrapToPiLocal(q1_down); wrapToPiLocal(q2_down)];
    q_all  = [q_up, q_down];

    if elbow == "up"
        q = q_up;
    elseif elbow == "down"
        q = q_down;
    else
        error('elbow must be "up" or "down".');
    end

    % ---------- Differential kinematics ----------
    % Jacobian for your FK:
    % x = -l1*sin(q1) - l2*sin(q1+q2)
    % y =  l1*cos(q1) + l2*cos(q1+q2)
    q1 = q(1);
    q2 = q(2);
    s1  = sin(q1);   c1  = cos(q1);
    s12 = sin(q1+q2); c12 = cos(q1+q2);

    J = [ -l1*c1 - l2*c12,   -l2*c12;
          -l1*s1 - l2*s12,   -l2*s12 ];

    % Damped least squares inverse for dq
    dq = solveDLS(J, v);
    %dq = J_inv(q, config)*v;
    %dq = J \ v;

    % Jdot * dq term
    dq1 = dq(1);
    dq2 = dq(2);
    w12 = dq1 + dq2;

    % Time derivatives of J entries
    % d/dt(-l1*cos(q1) - l2*cos(q1+q2)) =  l1*sin(q1)*dq1 + l2*sin(q1+q2)*(dq1+dq2)
    J11dot =  l1*s1*dq1 + l2*s12*w12;
    % d/dt(-l2*cos(q1+q2)) = l2*sin(q1+q2)*(dq1+dq2)
    J12dot =  l2*s12*w12;

    % d/dt(-l1*sin(q1) - l2*sin(q1+q2)) = -l1*cos(q1)*dq1 - l2*cos(q1+q2)*(dq1+dq2)
    J21dot = -l1*c1*dq1 - l2*c12*w12;
    % d/dt(-l2*sin(q1+q2)) = -l2*cos(q1+q2)*(dq1+dq2)
    J22dot = -l2*c12*w12;

    Jdot = [J11dot, J12dot;
            J21dot, J22dot];

    if hasAcc
        % ddq from: a = J*ddq + Jdot*dq  -> ddq = J^{-1}*(a - Jdot*dq)
        rhs = a - Jdot*dq;
        ddq = solveDLS(J, rhs);
        %ddq = J_inv(q, config)*rhs;
        %ddq = J\rhs;
    else
        ddq = zeros(2,1);  % or NaN(2,1) if you prefer to catch unintended use
    end
end

function x = solveDLS(J, b)
% Stable solve for x in J*x = b using damped least squares
% x = (J'*J + lambda^2 I)^(-1) J' b
    lambda = 1e-6; % tune if you see noise amplification near singularities
    A = J.'*J + (lambda*lambda)*eye(2);
    x = A \ (J.'*b);
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
