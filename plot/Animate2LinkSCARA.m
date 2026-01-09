function Animate2LinkSCARA(plotData, config)
% Animate2LinkSCARA  Real-time animation of a planar 2-link SCARA robot,
% with end-effector trajectory plotting.
%
% Requirements:
%   - plotData.q1, plotData.q2 are already strided (same length)
%   - plotData.step exists (stride used)
%   - config.sim.var.time_step exists (base simulation sample time)
%
% Optional (tracking mode only):
%   - plotData.q1_d, plotData.q2_d: desired joint trajectories
%     If q*_d are scalars or constant signals (regulation), desired EE path
%     is NOT plotted.

    % --- Link lengths ---
    if ~isfield(config,'robot') || ~isfield(config.robot,'geom') || ...
       ~isfield(config.robot.geom,'l1') || ~isfield(config.robot.geom,'l2')
        error("Animate2LinkSCARA: config.robot.geom.l1/l2 are missing.");
    end
    l1 = config.robot.geom.l1;
    l2 = config.robot.geom.l2;

    % --- Frame timing ---
    if ~isfield(config,'sim') || ~isfield(config.sim,'var') || ~isfield(config.sim.var,'time_step')
        error("Animate2LinkSCARA: config.sim.var.time_step is missing.");
    end
    Ts = config.sim.var.time_step;

    if ~isfield(plotData,'step') || isempty(plotData.step)
        error("Animate2LinkSCARA: plotData.step is missing.");
    end
    stride = plotData.step;

    dt_frame = Ts * stride;

    % --- Strided states ---
    if ~isfield(plotData,'q1') || ~isfield(plotData,'q2')
        error("Animate2LinkSCARA: plotData.q1 and plotData.q2 are required.");
    end
    q1 = plotData.q1(:);
    q2 = plotData.q2(:);

    N = min(numel(q1), numel(q2));
    q1 = q1(1:N);
    q2 = q2(1:N);

    % --- Precompute EE trajectory (actual) ---
    th1_all  = q1;
    th12_all = q1 + q2;

    x1_all = -l1 .* sin(th1_all);
    y1_all =  l1 .* cos(th1_all);

    x2_all = x1_all - l2 .* sin(th12_all);
    y2_all = y1_all + l2 .* cos(th12_all);

    % --- Optional: desired trajectory (plot only if tracking, not regulation) ---
    hasDesired = isfield(plotData,'q1_d') && isfield(plotData,'q2_d') && ...
                 ~isempty(plotData.q1_d) && ~isempty(plotData.q2_d);

    plotDesired = false;

    if hasDesired
        q1d_raw = plotData.q1_d;
        q2d_raw = plotData.q2_d;

        % Regulation detection: scalar OR (vector but constant)
        if isscalar(q1d_raw) || isscalar(q2d_raw)
            plotDesired = false;
        else
            q1d_vec = q1d_raw(:);
            q2d_vec = q2d_raw(:);

            % Guard: if too short, treat as regulation
            if numel(q1d_vec) < 2 || numel(q2d_vec) < 2
                plotDesired = false;
            else
                isConst1 = (max(q1d_vec) - min(q1d_vec)) == 0;
                isConst2 = (max(q2d_vec) - min(q2d_vec)) == 0;
                plotDesired = ~(isConst1 && isConst2);
            end
        end

        if plotDesired
            q1d = q1d_raw(:);
            q2d = q2d_raw(:);

            Nd = min([numel(q1d), numel(q2d), N]);
            q1d = q1d(1:Nd);
            q2d = q2d(1:Nd);

            th1d_all  = q1d;
            th12d_all = q1d + q2d;

            x1d_all = -l1 .* sin(th1d_all);
            y1d_all =  l1 .* cos(th1d_all);

            x2d_all = x1d_all - l2 .* sin(th12d_all);
            y2d_all = y1d_all + l2 .* cos(th12d_all);
        end
    end

    % --- Figure setup ---
    figTag  = "SCARA_Animation_Fig";
    figName = "2-Link SCARA Animation";

    fig = findall(groot, 'Type','figure', 'Tag', figTag);
    if isempty(fig)
        fig = figure('Name', figName, 'NumberTitle','off', 'Tag', figTag);
    else
        figure(fig); % bring it to front
    end

    clf(fig);
    ax = axes(fig);
    hold(ax,'on'); grid(ax,'on');
    axis(ax,'equal');

    border = 100e-3; 
    xlim(ax, [-933e-3 - border, 1015e-3 + border]);
    ylim(ax, [-631.50e-3 - border, l1 + l2 + border]);

    xlabel(ax,'x [m]');
    ylabel(ax,'y [m]');
    title(ax,'2-Link SCARA, real-time animation with EE trajectory');

    % Base marker
    plot(ax, 0, 0, 'k.', 'MarkerSize', 14);
    
        % -------------------------
    % Static background geometry
    % -------------------------
    hold(ax,'on');

    % 1) Rectangle: width 933, height 495
    % "right corner should be at origin" -> interpret as top-right corner at (0,0)
    % so rectangle spans x in [-933, 0], y in [-495, 0]
    rectW = 933e-3;
    rectH = 495e-3;
    rectPos = [-rectW, -rectH, rectW, rectH];  % [x y width height] with top-right at (0,0)
    rectangle(ax, 'Position', rectPos, 'EdgeColor', 'k', 'LineWidth', 1.5);

    % 2) Circles (radius 273) at given centers
    r = 273e-3/2;

    c1 = [-192.19e-3, -495e-3];
    viscircles(ax, c1, r, 'Color', 'k', 'LineWidth', 1.5);

    c2 = [-740.81e-3, -495e-3];
    viscircles(ax, c2, r, 'Color', 'k', 'LineWidth', 1.5);

    % 3) Straight line from (-933, -431.50) to (933, -431.50)
    yLine = -631.50e-3;
    plot(ax, [-933e-3, 933e-3], [yLine, yLine], 'k-', 'LineWidth', 1.5);

    % Optional: keep these shapes behind the robot and traces
    % (MATLAB usually draws later plots on top, but this ensures ordering)
    % uistack(findobj(ax,'Type','rectangle'),'bottom'); % rectangle only

    % --- Plot trajectories (actual always, desired only if tracking) ---
    hPathAll = plot(ax, x2_all, y2_all, '-', 'LineWidth', 1);      % context
    hTrace   = plot(ax, x2_all(1), y2_all(1), '-', 'LineWidth', 2); % grows

    legendHandles = [hPathAll, hTrace];
    legendEntries = {'EE path (actual)', 'EE trace (so far)'};

    if hasDesired && plotDesired
        hPathDes = plot(ax, x2d_all, y2d_all, '--', 'LineWidth', 1.5);
        legendHandles(end+1) = hPathDes;
        legendEntries{end+1} = 'EE path (desired)';
    end

    legend(ax, legendHandles, legendEntries, 'Location','best');

    % Link line objects
    hLink1 = plot(ax, [0 0], [0 0], 'LineWidth', 3);
    hLink2 = plot(ax, [0 0], [0 0], 'LineWidth', 3);

    % End-effector marker
    hEE = plot(ax, x2_all(1), y2_all(1), 'o', 'MarkerSize', 6, 'LineWidth', 1.5);

    % Text overlay
    hTxt = text(ax, 0.02, 0.98, "", 'Units','normalized', ...
        'HorizontalAlignment','left', 'VerticalAlignment','top');

    % Improve responsiveness
    set(fig, 'Renderer', 'opengl');
    
    % --- Animation loop ---
    for k = 1:N
        if ~isvalid(fig) || ~isvalid(ax)
            break;
        end

        % Use precomputed kinematics
        x1 = x1_all(k); y1 = y1_all(k);
        x2 = x2_all(k); y2 = y2_all(k);

        % Update robot graphics
        set(hLink1, 'XData', [0 x1],  'YData', [0 y1]);
        set(hLink2, 'XData', [x1 x2], 'YData', [y1 y2]);
        set(hEE,    'XData', x2,      'YData', y2);

        % Update trace-so-far
        set(hTrace, 'XData', x2_all(1:k), 'YData', y2_all(1:k));

        % Update overlay text
        hTxt.String = sprintf("t=%.3f/%.3f s, q1=%.3f rad, q2=%.3f rad", k*dt_frame, N*dt_frame, q1(k), q2(k));

        drawnow limitrate;

        % Real-time pacing
        pause(dt_frame);
    end

    hold(ax,'off');
end
