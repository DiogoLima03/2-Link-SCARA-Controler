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

    xlabel(ax,'x [m]');
    ylabel(ax,'y [m]');
    title(ax,'2-Link SCARA, real-time animation with EE trajectory');

    % Base marker
    plot(ax, 0, 0, 'k.', 'MarkerSize', 14);

    % -------------------------
    % Static background geometry
    % -------------------------
    % 1) Rectangle: width 933, height 495
    % "right corner should be at origin" -> top-right corner at (0,0)
    rectW = 933e-3;
    rectH = 495e-3;
    rectPos = [-rectW, -rectH, rectW, rectH];  % [x y w h]
    rectangle(ax, 'Position', rectPos, 'EdgeColor', 'k', 'LineWidth', 1.5);

    % 2) Circles (radius 273 mm) at given centers
    r = 273e-3/2;
    c1 = [-192.19e-3, -495e-3];
    viscircles(ax, c1, r, 'Color', 'k', 'LineWidth', 1.5);

    c2 = [-740.81e-3, -495e-3];
    viscircles(ax, c2, r, 'Color', 'k', 'LineWidth', 1.5);

    % 3) Straight line (using your current code value)
    yLine = -631.50e-3;
    plot(ax, [-933e-3, 933e-3], [yLine, yLine], 'k-', 'LineWidth', 1.5);

    % --- Plot trajectories (actual always, desired only if tracking) ---
    hPathAll = plot(ax, x2_all, y2_all, '-', 'LineWidth', 1);      % context
    hTrace   = plot(ax, x2_all(1), y2_all(1), '-', 'LineWidth', 2); % grows

    legendHandles = [hPathAll, hTrace];
    legendEntries = {'EE path (actual)', 'EE trace (so far)'};

    if hasDesired && plotDesired
        hPathDes = plot(ax, x2d_all, y2d_all, '--', 'LineWidth', 1.5);
        legendHandles(end+1) = hPathDes; %#ok<AGROW>
        legendEntries{end+1} = 'EE path (desired)'; %#ok<AGROW>
    end

    legend(ax, legendHandles, legendEntries, 'Location','best');

    % ----------------------------
    % Smart limits (NOW works, because geometry/paths exist)
    % ----------------------------
    border = 100e-3;

    xlim_default = [-933e-3, 1015e-3] + [-border, border];
    ylim_default = [-631.50e-3, l1 + l2] + [-border, border];

    % Bounds from known geometry + trajectories (more robust than ax.Children)
    xAll = [ ...
        0; ...
        -933e-3; 933e-3; ...
        -933e-3; 0; ...
        (c1(1) + r); (c1(1) - r); ...
        (c2(1) + r); (c2(1) - r); ...
        x2_all(:) ...
    ];

    yAll = [ ...
        0; ...
        yLine; yLine; ...
        -495e-3; 0; ...
        (c1(2) + r); (c1(2) - r); ...
        (c2(2) + r); (c2(2) - r); ...
        y2_all(:) ...
    ];

    if hasDesired && plotDesired
        xAll = [xAll; x2d_all(:)];
        yAll = [yAll; y2d_all(:)];
    end

    xAll = xAll(isfinite(xAll));
    yAll = yAll(isfinite(yAll));

    xmin = min(xAll); xmax = max(xAll);
    ymin = min(yAll); ymax = max(yAll);

    xlim_final = xlim_default;
    ylim_final = ylim_default;

    if xmin < xlim_default(1) || xmax > xlim_default(2)
        xlim_final = [min(xlim_default(1), xmin - border), max(xlim_default(2), xmax + border)];
    end
    if ymin < ylim_default(1) || ymax > ylim_default(2)
        ylim_final = [min(ylim_default(1), ymin - border), max(ylim_default(2), ymax + border)];
    end

    xlim(ax, xlim_final);
    ylim(ax, ylim_final);

    neonGreen = [0.0, 1.0, 0.35];

    % Link line objects
    hLink1 = plot(ax, [0 0], [0 0], 'LineWidth', 3);
    hLink2 = plot(ax, [0 0], [0 0], 'LineWidth', 3, 'Color', neonGreen); % <-- add back
    hEE    = plot(ax, x2_all(1), y2_all(1), 'o', 'MarkerSize', 6, 'LineWidth', 1.5); % <-- add back


    % --- Visual tweak parameters ---
    link2_cut = 0.15;           % cut 20% of link2 for drawing (visual only)
    alpha2    = 1 - link2_cut;  % 0.8 -> draw 80% of link2
    
    grip_width  = 0.10;         % [m] distance between fingers (tune)
    grip_length = 0.10;         % [m] finger length (tune)
    
    % --- Replace hLink2 with a shortened visual link2 line ---
    % (keep hLink2 handle name if you want, or use a new one)
    % hLink2 already exists in your code, so we will reuse it as "short link2".
    
    % --- Original end-effector point marker (at original x2,y2) ---
    hTip = plot(ax, x2_all(1), y2_all(1), 'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', neonGreen, ...
    'MarkerEdgeColor', 'k');

    % --- Gripper line objects: two fingers + crossbar (|__|) ---
    hGripL = plot(ax, [0 0], [0 0], ...
        'LineWidth', 2, ...
        'Color', neonGreen);
    
    hGripR = plot(ax, [0 0], [0 0], ...
        'LineWidth', 2, ...
        'Color', neonGreen);
    
    hGripB = plot(ax, [0 0], [0 0], ...
        'LineWidth', 2, ...
        'Color', neonGreen);

    
    % Optional: If you want the EE marker to be the "shortened link end" instead,
    % you can keep hEE where it is. Here we keep hEE at the original x2,y2 as before.

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
    
        % Use precomputed kinematics (true joint and EE positions)
        x1 = x1_all(k); y1 = y1_all(k);
        x2 = x2_all(k); y2 = y2_all(k);
    
        % --- Shortened visual endpoint for link2 (80% of the way from joint2 to EE) ---
        x2s = x1 + alpha2*(x2 - x1);
        y2s = y1 + alpha2*(y2 - y1);
    
        % Update link1 (full)
        set(hLink1, 'XData', [0 x1],  'YData', [0 y1]);
    
        % Update link2 (shortened for drawing)
        set(hLink2, 'XData', [x1 x2s], 'YData', [y1 y2s]);
    
        % Keep EE marker at the ORIGINAL tip (x2,y2)
        set(hEE, 'XData', x2, 'YData', y2);
    
        % Point marker at original tip (explicit)
        set(hTip, 'XData', x2, 'YData', y2);
    
        % --- Gripper at the ORIGINAL tip, oriented with link2 direction ---
        dx = x2 - x1;
        dy = y2 - y1;
        L  = hypot(dx, dy);
    
        if L < 1e-9
            % Degenerate case, hide gripper
            set(hGripL, 'XData', [NaN NaN], 'YData', [NaN NaN]);
            set(hGripR, 'XData', [NaN NaN], 'YData', [NaN NaN]);
            set(hGripB, 'XData', [NaN NaN], 'YData', [NaN NaN]);
        else
            % Unit vectors: u along link2, v perpendicular
            ux = dx / L;  uy = dy / L;
            vx = -uy;     vy = ux;
    
            % Build a "|__|" shape:
            % Two fingers start at the tip, offset sideways by +/- width/2,
            % and extend backward along -u by grip_length.
            w2 = grip_width / 2;
    
            % Finger base points (near the tip)
            pL0 = [x2 + vx*w2, y2 + vy*w2];
            pR0 = [x2 - vx*w2, y2 - vy*w2];
    
            % Finger end points (extend backward along -u)
            pL1 = [pL0(1) - ux*grip_length, pL0(2) - uy*grip_length];
            pR1 = [pR0(1) - ux*grip_length, pR0(2) - uy*grip_length];
    
            % Update finger lines
            set(hGripL, 'XData', [pL0(1) pL1(1)], 'YData', [pL0(2) pL1(2)]);
            set(hGripR, 'XData', [pR0(1) pR1(1)], 'YData', [pR0(2) pR1(2)]);
    
            % Bottom bar connecting the finger ends (the "__" part)
            set(hGripB, 'XData', [pL1(1) pR1(1)], 'YData', [pL1(2) pR1(2)]);
        end
    
        % Update trace-so-far (still using original EE)
        set(hTrace, 'XData', x2_all(1:k), 'YData', y2_all(1:k));
    
        % Update overlay text
        hTxt.String = sprintf("t=%.3f/%.3f s, q1=%.3f rad, q2=%.3f rad", ...
                              k*dt_frame, N*dt_frame, q1(k), q2(k));
    
        drawnow limitrate;
        pause(dt_frame);
    end

    hold(ax,'off');
end
