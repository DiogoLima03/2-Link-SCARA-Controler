function PlotInput(plotData, config)

    % Figure
    figTag  = "PlotInput_Fig";
    figName = "Input torque";

    fig = findall(groot, 'Type','figure', 'Tag', figTag);
    if isempty(fig)
        fig = figure('Name', figName, 'NumberTitle','off', 'Tag', figTag);
    else
        figure(fig);
    end

    clf(fig);

    % Strided time vector
    t = plotData.t(plotData.idx);

    % Checks
    if ~isfield(plotData,'u1') || ~isfield(plotData,'u2')
        error("PlotInput: plotData.u1/u2 not found. Extract them before calling PlotInput.");
    end
    if numel(plotData.u1) ~= numel(t) || numel(plotData.u2) ~= numel(t)
        error("PlotInput: Length mismatch, t=%d, u1=%d, u2=%d.", numel(t), numel(plotData.u1), numel(plotData.u2));
    end

    u1 = plotData.u1(:);
    u2 = plotData.u2(:);

    % Determine if saturation mode is active
    satActive = false;
    if isfield(config,'sim') && isfield(config.sim,'mode') && isfield(config.sim.mode,'inputSaturation')
        satActive = (config.sim.mode.inputSaturation == InputSaturation.Active);
    end

    % Limits
    if satActive
        u1_max = config.robot.motor.saturation.joint_1;
        u2_max = config.robot.motor.saturation.joint_2;
        u1_min = -u1_max;
        u2_min = -u2_max;

        % Identify exceedance points (commanded torque outside limits)
        ex1 = (u1 > u1_max) | (u1 < u1_min);
        ex2 = (u2 > u2_max) | (u2 < u2_min);
    else
        ex1 = false(size(u1));
        ex2 = false(size(u2));
    end

    % Plot
    ax = axes(fig); %#ok<LAXES>
    hold(ax,'on'); grid(ax,'on');

    % Plot full signals (base)
    h1 = plot(ax, t, u1, 'LineWidth', 1.5, 'Color', "b");
    h2 = plot(ax, t, u2, 'LineWidth', 1.5, 'Color', "g");

    % Overlay exceedance points in red (only when saturation active)
    if satActive
        if any(ex1)
            plot(ax, t(ex1), u1(ex1), 'r.', 'MarkerSize', 10);
        end
        if any(ex2)
            plot(ax, t(ex2), u2(ex2), 'r.', 'MarkerSize', 10);
        end

        % Draw saturation bounds
        yline(ax, u1_max, '--', '\tau_{1,max}', 'LabelHorizontalAlignment','left');
        yline(ax, u1_min, '--', '\tau_{1,min}', 'LabelHorizontalAlignment','left');
        yline(ax, u2_max, '--', '\tau_{2,max}', 'LabelHorizontalAlignment','left');
        yline(ax, u2_min, '--', '\tau_{2,min}', 'LabelHorizontalAlignment','left');
    end

    xlabel(ax, 'Time [s]');
    ylabel(ax, 'Torque [N·m]');
    title(ax, 'Input torques');

    % Legend with click-to-toggle
    lg = legend(ax, [h1 h2], {'\tau_1','\tau_2'}, 'Location', 'best');
    set(lg, 'ItemHitFcn', @legendItemToggle);

    hold(ax,'off');
end

function legendItemToggle(~, event)
% Toggle visibility of a plotted object by clicking its legend entry.
    obj = event.Peer;
    if strcmp(obj.Visible,'on')
        obj.Visible = 'off';
    else
        obj.Visible = 'on';
    end
end
