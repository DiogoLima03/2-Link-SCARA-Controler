plotData.t = out.x.time;
plotData.x = out.x.signals.values;

% --- Stride (downsample for plotting only doesnt always work) ---
plotData.N = numel(plotData.t);

%plotData.targetPts = 50000;                 % 10k–100k is typical, tune as needed
%plotData.step = max(1, floor(plotData.N/plotData.targetPts)); % stride
plotData.step = 1;
plotData.idx = 1:plotData.step:plotData.N; % indices to keep for plotting

%% States 

% Make it robust to either shape
if ndims(plotData.x) == 3
    plotData.q1      = squeeze(plotData.x(1,1,:));
    plotData.q2      = squeeze(plotData.x(2,1,:));
    plotData.dot_q1  = squeeze(plotData.x(3,1,:));
    plotData.dot_q2  = squeeze(plotData.x(4,1,:));
else
    plotData.q1      = plotData.x(1,:).';
    plotData.q2      = plotData.x(2,:).';
    plotData.dot_q1  = plotData.x(3,:).';
    plotData.dot_q2  = plotData.x(4,:).';
end

% Apply stride (for plotting only)
plotData.q1      = plotData.q1(plotData.idx);
plotData.q2      = plotData.q2(plotData.idx);
plotData.dot_q1  = plotData.dot_q1(plotData.idx);
plotData.dot_q2  = plotData.dot_q2(plotData.idx);

%% Estimated states (From State Reconstruction or Observer)
plotData.x_est = out.x_est.signals.values;

if ndims(plotData.x_est) == 3
    plotData.q1_est     = squeeze(plotData.x_est(1,1,:));
    plotData.q2_est     = squeeze(plotData.x_est(2,1,:));
    plotData.dot_q1_est = squeeze(plotData.x_est(3,1,:));
    plotData.dot_q2_est = squeeze(plotData.x_est(4,1,:));
else
    plotData.q1_est     = plotData.x_est(1,:).';
    plotData.q2_est     = plotData.x_est(2,:).';
    plotData.dot_q1_est = plotData.x_est(3,:).';
    plotData.dot_q2_est = plotData.x_est(4,:).';
end

plotData.q1_est     = plotData.q1_est(plotData.idx);
plotData.q2_est     = plotData.q2_est(plotData.idx);
plotData.dot_q1_est = plotData.dot_q1_est(plotData.idx);
plotData.dot_q2_est = plotData.dot_q2_est(plotData.idx);


%% Desired / ref States 
plotData.x_d = out.x_d.signals.values;

if config.sim.mode.refGeneration == RefGeneration.Trajectory
    if ndims(plotData.x_d) == 3
        plotData.q1_d      = squeeze(plotData.x_d(1,1,:));
        plotData.q2_d      = squeeze(plotData.x_d(2,1,:));
        plotData.dot_q1_d  = squeeze(plotData.x_d(3,1,:));
        plotData.dot_q2_d  = squeeze(plotData.x_d(4,1,:));
    else
        plotData.q1_d      = plotData.x_d(1,:).';
        plotData.q2_d      = plotData.x_d(2,:).';
        plotData.dot_q1_d  = plotData.x_d(3,:).';
        plotData.dot_q2_d  = plotData.x_d(4,:).';
    end

    plotData.q1_d      = plotData.q1_d(plotData.idx);
    plotData.q2_d      = plotData.q2_d(plotData.idx);
    plotData.dot_q1_d  = plotData.dot_q1_d(plotData.idx);
    plotData.dot_q2_d  = plotData.dot_q2_d(plotData.idx);
elseif config.sim.mode.refGeneration == RefGeneration.Regulation
    plotData.q1_d = plotData.x_d(1,1);
    plotData.q2_d = plotData.x_d(1,2);
    plotData.dot_q1_d = plotData.x_d(1,3);
    plotData.dot_q2_d = plotData.x_d(1,4);
else
    error("plotDatating: Incompatible config: refGeneration")
end

%% I controller values (robust) - store full vectors + time

plotData.e_i        = [];
plotData.e_int_q_1  = [];
plotData.e_int_q_2  = [];
plotData.e_i_t      = [];   % time base of e_i, if available

% --- Robust detection and retrieval of e_i (NO hasVariable) ---
plotData.has_ei = false;

if isstruct(out)
    if isfield(out,'e_i') && ~isempty(out.e_i)
        plotData.has_ei = true;
        plotData.ei = out.e_i;
    end

else
    % Works for Simulink.SimulationOutput and avoids hasVariable entirely
    try
        plotData.ei = out.get('e_i');        % if 'out' is SimulationOutput and e_i exists
        plotData.has_ei = ~isempty(plotData.ei);
    catch
        plotData.has_ei = false;
        plotData.ei = [];
    end
end

if plotData.has_ei 
    plotData.e_i = out.e_i.signals.values;
    
    % Capture time if present (Structure with Time often has it)
    if isfield(out.e_i,'time')
        plotData.e_i_t = out.e_i.time;
    end

    if ndims(plotData.e_i) == 3
        plotData.e_int_q_1 = squeeze(plotData.e_i(1,1,:));
        plotData.e_int_q_2 = squeeze(plotData.e_i(2,1,:));
    else
        plotData.e_int_q_1 = plotData.e_i(1,:).';
        plotData.e_int_q_2 = plotData.e_i(2,:).';
    end

    % Ensure column vectors
    plotData.e_int_q_1 = plotData.e_int_q_1(:);
    plotData.e_int_q_2 = plotData.e_int_q_2(:);
else
    % No e_i found, leave empties
end

%% Input torque

plotData.u = out.u.signals.values;

if ndims(plotData.u) == 3
    plotData.u1     = squeeze(plotData.u(1,1,:));
    plotData.u2     = squeeze(plotData.u(2,1,:));
else
    plotData.u1     = plotData.u(1,:).';
    plotData.u2     = plotData.u(2,:).';
end

plotData.u1     = plotData.u1(plotData.idx);
plotData.u2     = plotData.u2(plotData.idx);

%% plotDatating: States, Estimated States and Desired States

PlotStatesEstDes(plotData);

%% Plooting: Errors

PlotError(plotData, config);

%% PLot Input Torque

PlotInput(plotData, config);

%% plotDatating: 2-Link SCARA Animation

Animate2LinkSCARA(plotData, config);

%% Checking the noise measurement - Cheking the covariance matrices

% plotData.measNoise.encoders = out.measNoiseEncoders.signals.values;
% 
% 
% X = plotData.measNoise.encoders;   % 755 x 2
% mu = mean(X, 1)        % 1 x 2
% sigma = std(X, 0, 1)   % 1 x 2 (unbiased, normalized by N-1)
% sigma_pop = std(X, 1, 1)
% %% 
% Sigma = cov(X)
% %%
% R = corrcoef(X)
%
% N = 1000;
% 
% sigma1 = 0.002;
% sigma2 = 0.002;
% rho    = 0.3;
% 
% Sigma = [ sigma1^2, rho*sigma1*sigma2;
%           rho*sigma1*sigma2, sigma2^2 ];
% 
% L = chol(Sigma, 'lower');
% 
% u = randn(N,2);      % uncorrelated, unit variance
% v = u * L.';         % correlated signals
% 
% % Check
% cov(v)
% corrcoef(v)
