function [ref_d_TS, seg_id] = trag_gen(t, config)
%#codegen
% Global time-scaling along a centripetal Catmull–Rom spline through waypoints.
% Velocity is zero only at the start and end (not at every waypoint).

% =========================
% USER PARAMETERS
% =========================
max_vel  = config.refGeneration.trajectory.max_vel;         % [m/s] global max (see note below about safe zones)
max_acc  = config.refGeneration.trajectory.max_acc;         % [m/s^2]
time_scale = config.refGeneration.trajectory.time_scale;    % >= 1.0, increase if you see saturation
alpha = config.refGeneration.trajectory.alpha;              % 0.5 centripetal Catmull–Rom

W = config.refGeneration.trajectory.W;
Npt  = size(W,2);   
Nseg = Npt - 1;     

% =========================
% PRECOMPUTE cumulative chord lengths (for global progress mapping)
% =========================
Lseg = zeros(Nseg,1);
Lcum = zeros(Nseg+1,1); % Lcum(1)=0
for k = 1:Nseg
    d = W(:,k+1) - W(:,k);
    Lseg(k) = sqrt(d(1)^2 + d(2)^2);
    if Lseg(k) < 1e-9
        Lseg(k) = 1e-9;
    end
    Lcum(k+1) = Lcum(k) + Lseg(k);
end
Ltotal = Lcum(end);

% =========================
% GLOBAL DURATION from quintic time law bounds (directional limiting)
% For quintic z(tau): max(z_dot)=1.875/T, max(z_ddot)=7.5/T^2
% If we treat progress as distance g in [0,Ltotal], then:
% g_dot_max = 1.875*Ltotal/T  <= max_vel  -> T >= 1.875*Ltotal/max_vel
% g_ddot_max= 7.5*Ltotal/T^2 <= max_acc  -> T >= sqrt(7.5*Ltotal/max_acc)
% =========================
Tv = (1.875 * Ltotal) / max(max_vel, 1e-9);
Ta = sqrt((7.5 * Ltotal) / max(max_acc, 1e-9));
Ttotal = time_scale * max(Tv, Ta);
if Ttotal < 1e-3
    Ttotal = 1e-3;
end

% =========================
% GLOBAL QUINTIC progress z in [0,1]
% =========================
if t <= 0
    tau = 0;
elseif t >= Ttotal
    tau = 1;
else
    tau = t / Ttotal;
end

z    = 10*tau^3 - 15*tau^4 + 6*tau^5;
zdot = (30*tau^2 - 60*tau^3 + 30*tau^4) / Ttotal;
zdd  = (60*tau  - 180*tau^2 + 120*tau^3) / (Ttotal*Ttotal);

% Global distance progress g(t) in [0, Ltotal]
g    = z    * Ltotal;
gdot = zdot * Ltotal;
gdd  = zdd  * Ltotal;

% =========================
% Map g to active segment k and local fraction u in [0,1]
% =========================
if g <= 0
    kseg = 1;
elseif g >= Ltotal
    kseg = Nseg;
else
    kseg = 1;
    for k = 1:Nseg
        if g < Lcum(k+1)
            kseg = k;
            break;
        end
    end
end
seg_id = kseg;

g0 = Lcum(kseg);
u  = (g - g0) / Lseg(kseg);
if u < 0, u = 0; end
if u > 1, u = 1; end

% Because u depends linearly on g in this segment:
% u_dot = g_dot / Lseg, u_ddot = g_ddot / Lseg
udot = gdot / Lseg(kseg);
udd  = gdd  / Lseg(kseg);

% =========================
% CENTRIPETAL CATMULL–ROM on segment kseg: Pk -> Pk+1
% Uses 4 points: P_{i-1}, P_i, P_{i+1}, P_{i+2} with endpoint clamping
% =========================
i = kseg;

P_im1 = getPt(W, i-1);
P_i   = getPt(W, i);
P_ip1 = getPt(W, i+1);
P_ip2 = getPt(W, i+2);

% Parameter times based on distances^alpha
t0 = 0;
t1 = t0 + distAlpha(P_im1, P_i,   alpha);
t2 = t1 + distAlpha(P_i,   P_ip1, alpha);
t3 = t2 + distAlpha(P_ip1, P_ip2, alpha);

% Local CR parameter s in [t1, t2] driven by u
s     = t1 + u*(t2 - t1);
dsdu  = (t2 - t1);
sdot  = dsdu * udot;
sddot = dsdu * udd;

% Evaluate p(s), dp/ds, d2p/ds2
[p, ps, pss] = catmullRomCentripetal(P_im1,P_i,P_ip1,P_ip2, t0,t1,t2,t3, s);

% Chain rule
vel = ps  * sdot;
acc = pss * (sdot*sdot) + ps * sddot;

ref_d_TS = [p(1); p(2); vel(1); vel(2); acc(1); acc(2)];
end

% ===== helper functions (codegen friendly) =====

function P = getPt(W, idx)
n = size(W,2);
if idx < 1
    P = W(:,1);
elseif idx > n
    P = W(:,n);
else
    P = W(:,idx);
end
end

function d = distAlpha(A,B,alpha)
v = B - A;
d0 = sqrt(v(1)^2 + v(2)^2);
d = max(d0, 1e-9)^alpha;
end

function [p, dpds, d2pds2] = catmullRomCentripetal(P0,P1,P2,P3, t0,t1,t2,t3, s)
% Barry–Goldman form (stable). Returns p(s), dp/ds, d2p/ds2.

A1  = lerp(P0,P1, t0,t1, s);  A1s  = dlerp(P0,P1, t0,t1);  A1ss = zeros(2,1);
A2  = lerp(P1,P2, t1,t2, s);  A2s  = dlerp(P1,P2, t1,t2);  A2ss = zeros(2,1);
A3  = lerp(P2,P3, t2,t3, s);  A3s  = dlerp(P2,P3, t2,t3);  A3ss = zeros(2,1);

B1  = lerp(A1,A2, t0,t2, s);
B2  = lerp(A2,A3, t1,t3, s);

[B1s, B1ss] = dlerp2(A1,A2,A1s,A2s,A1ss,A2ss, t0,t2, s);
[B2s, B2ss] = dlerp2(A2,A3,A2s,A3s,A2ss,A3ss, t1,t3, s);

p   = lerp(B1,B2, t1,t2, s);
[dpds, d2pds2] = dlerp2(B1,B2,B1s,B2s,B1ss,B2ss, t1,t2, s);
end

function X = lerp(A,B, ta,tb, s)
u = (s - ta) / (tb - ta);
X = (1 - u)*A + u*B;
end

function Xs = dlerp(A,B, ta,tb)
Xs = (B - A) / (tb - ta);
end

function [Xs, Xss] = dlerp2(A,B, As,Bs, Ass,Bss, ta,tb, s)
u  = (s - ta) / (tb - ta);
ud = 1/(tb - ta);
Xs  = -(ud)*A + (1-u)*As + (ud)*B + u*Bs;
Xss = -ud*As + (1-u)*Ass + ud*Bs + u*Bss;
end
