function [m, Iz] = RectangleHollowLink(b, h, t_b, t_h, L, rho)
% hollowRectLinkInertia
%   Computes mass and exact mass moment of inertia Iz (about z-axis through COM)
%   for a hollow rectangular tube link.
%
%   Inputs:
%       b   : outer width  [m]
%       h   : outer height [m]
%       b1  : inner width  [m]
%       h1  : inner height [m]
%       L   : link length  [m]
%       rho : material density [kg/m^3]
%
%   Outputs:
%       m   : mass of the link [kg]
%       Iz  : mass moment of inertia about z-axis through COM [kg·m^2]
%
%   Geometry is assumed to be symmetric around the centroid.
    
    b1 = b-2*t_b;
    h1 = h-2*t_h;

    % Cross-sectional areas
    A_outer = b  * h;
    A_inner = b1 * h1;

    % Mass
    m = rho * L * (A_outer - A_inner);

    % Exact mass moment of inertia about z-axis through COM
    Iz = (rho * L / 12) * ( ...
          A_outer * (L^2 + b^2) ...
        - A_inner * (L^2 + b1^2) );
end
