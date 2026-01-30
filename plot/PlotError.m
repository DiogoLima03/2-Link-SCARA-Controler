function PlotError(plotData, config)
% PlotError  Plot position and velocity errors for q1 and q2 (actual and estimated).
% Legend entries can be clicked to show/hide each trace.

    % Strided time vector
    t = plotData.t(plotData.idx);

    % Build desired signals as vectors (works for scalar regulation too)
    q1_d      = makeVectorDesired(plotData.q1_d,      numel(t));
    q2_d      = makeVectorDesired(plotData.q2_d,      numel(t));
    dot_q1_d  = makeVectorDesired(plotData.dot_q1_d,  numel(t));
    dot_q2_d  = makeVectorDesired(plotData.dot_q2_d,  numel(t));

    % Errors (actual - desired)
    e_int_q1  = plotData.e_int_q_1;
    e_int_q2  = plotData.e_int_q_2;
    e_q1      = plotData.q1      - q1_d;
    e_q2      = plotData.q2      - q2_d;
    e_dot_q1  = plotData.dot_q1  - dot_q1_d;
    e_dot_q2  = plotData.dot_q2  - dot_q2_d;
    
    % Errors (estimated - desired)
    e_q1_est      = plotData.q1_est      - q1_d;
    e_q2_est      = plotData.q2_est      - q2_d;
    e_dot_q1_est  = plotData.dot_q1_est  - dot_q1_d;
    e_dot_q2_est  = plotData.dot_q2_est  - dot_q2_d;
    
    % Figure handling
    figTag  = "PlotError_Fig";
    figName = "Errors: Position, Velocity";

    fig = findall(groot, 'Type','figure', 'Tag', figTag);
    if isempty(fig)
        fig = figure('Name', figName, 'NumberTitle','off', 'Tag', figTag);
    else
        figure(fig);
    end

    clf(fig);
    
    % -------------------------
    % Position error subplot
    % -------------------------
    if (config.sim.mode.controllerConfig.action == ControllerAction.PID_zeta_omega) || ...
            (config.sim.mode.controllerConfig.action == ControllerAction.PID_LQR)

        ax0 = subplot(3,1,1);
        hold(ax0,'on');
    
        h_e_int_q1     = plot(ax0, t, e_int_q1,     'LineWidth', 1.2);
        h_e_int_q2     = plot(ax0, t, e_int_q2,     'LineWidth', 1.2);
        % h_e_q1_est = plot(ax0, t, e_q1_est, 'LineWidth', 1.2);
        % h_e_q2_est = plot(ax0, t, e_q2_est, 'LineWidth', 1.2);
    
        ylabel(ax0, 'Integral Error e_q_int [rad]', 'Interpreter','tex');
        title(ax0, 'Tracking Errors');
        grid(ax0,'on');
    
        lg0 = legend(ax0, ...
            [h_e_int_q1, h_e_int_q2 %, h_e_q1_est, h_e_q2_est...
            ], ...
            {'e_{q1}_int', 'e_{q2}_int'  ... %, 'e_{q1,est}', 'e_{q2,est}'
             }, ...
            'Interpreter','tex', ...
            'Location','best');
    
        % Make legend click-to-toggle robust
        set(lg0, 'ItemHitFcn', @legendItemToggle);
    
        hold(ax0,'off');
    end 

    % -------------------------
    % Position error subplot
    % -------------------------
    ax1 = subplot(3,1,2);
    hold(ax1,'on');

    h_e_q1     = plot(ax1, t, e_q1,     'LineWidth', 1.2);
    h_e_q2     = plot(ax1, t, e_q2,     'LineWidth', 1.2);
    h_e_q1_est = plot(ax1, t, e_q1_est, 'LineWidth', 1.2);
    h_e_q2_est = plot(ax1, t, e_q2_est, 'LineWidth', 1.2);

    ylabel(ax1, 'Proportional Error e_q [rad]', 'Interpreter','tex');
    %title(ax1, 'Tracking Errors');
    grid(ax1,'on');

    lg1 = legend(ax1, ...
        [h_e_q1, h_e_q2, h_e_q1_est, h_e_q2_est], ...
        {'e_{q1}', 'e_{q2}', 'e_{q1,est}', 'e_{q2,est}'}, ...
        'Interpreter','tex', ...
        'Location','best');

    % Make legend click-to-toggle robust
    set(lg1, 'ItemHitFcn', @legendItemToggle);

    hold(ax1,'off');

    % -------------------------
    % Velocity error subplot
    % -------------------------
    ax2 = subplot(3,1,3);
    hold(ax2,'on');

    h_e_dq1     = plot(ax2, t, e_dot_q1,     'LineWidth', 1.2);
    h_e_dq2     = plot(ax2, t, e_dot_q2,     'LineWidth', 1.2);
    h_e_dq1_est = plot(ax2, t, e_dot_q1_est, 'LineWidth', 1.2);
    h_e_dq2_est = plot(ax2, t, e_dot_q2_est, 'LineWidth', 1.2);

    ylabel(ax2, 'Derivative Error e_{dot q} [rad/s]', 'Interpreter','tex');
    grid(ax2,'on');

    lg2 = legend(ax2, ...
        [h_e_dq1, h_e_dq2, h_e_dq1_est, h_e_dq2_est], ...
        {'e_{dot q1}', 'e_{dot q2}', 'e_{dot q1,est}', 'e_{dot q2,est}'}, ...
        'Interpreter','tex', ...
        'Location','best');

    set(lg2, 'ItemHitFcn', @legendItemToggle);

    hold(ax2,'off');
end

function legendItemToggle(~, event)
% legendItemToggle  Toggle visibility of a plotted object by clicking its legend entry.
% Works for line/patch objects. Keeps legend entry visible while toggling the plot.

    obj = event.Peer;  % the graphics object associated with the legend item
    if strcmp(obj.Visible, 'on')
        obj.Visible = 'off';
    else
        obj.Visible = 'on';
    end
end

function y = makeVectorDesired(d, N)
% makeVectorDesired  Ensure desired signal is an Nx1 vector.
% If scalar, replicate. If vector, ensure column and correct length.
    if isscalar(d)
        y = repmat(d, N, 1);
    else
        y = d(:);
        if numel(y) ~= N
            error("Desired signal length (%d) does not match time length (%d).", numel(y), N);
        end
    end
end
