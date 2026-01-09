function PlotStatesEstDes(plotData)
    % Plotting: States, Estimated States and Desired States
    % Left y-axis: angles q [rad]
    % Right y-axis: angular velocities dq [rad/s]

    figTag  = "PlotStatesEstDes_Fig";
    figName = "States, Estimated States and Desired States";

    fig = findall(groot, 'Type','figure', 'Tag', figTag);
    if isempty(fig)
        fig = figure('Name', figName, 'NumberTitle','off', 'Tag', figTag);
    else
        figure(fig);
    end

    clf(fig);
    ax = axes(fig);
    hold(ax,'on');

    t = plotData.t(plotData.idx);

    % =========================
    % Style parameters
    % =========================
    lw_state = 1.6;
    lw_est   = 1.3;
    lw_des   = 1.3;

    ls = '-';
    mk = 'none';

    % -------------------------
    % Colors (unchanged)
    % -------------------------
    c_q1  = [0 0 1];
    c_q2  = [21 87 30]/255;
    c_dq1 = [190 80 20]/255;
    c_dq2 = [160 43 147]/255;

    c_q1_est  = [11 195 245]/255;
    c_q2_est  = [71 211 89]/255;
    c_dq1_est = [236 128 74]/255;
    c_dq2_est = [216 109 205]/255;

    c_q1_d  = [137 247 255]/255;
    c_q2_d  = [193 240 200]/255;
    c_dq1_d = [251 226 213]/255;
    c_dq2_d = [242 206 239]/255;

    % Legend storage
    h   = gobjects(0);
    leg = strings(0);

    % -------------------------
    % LEFT AXIS: angles
    % -------------------------
    yyaxis(ax,'left');

    h(end+1) = plot(ax,t,plotData.q1,'LineStyle',ls,'Marker',mk,'LineWidth',lw_state,'Color',c_q1);
    leg(end+1) = "q_1";

    h(end+1) = plot(ax,t,plotData.q1_est,'LineStyle',ls,'Marker',mk,'LineWidth',lw_est,'Color',c_q1_est);
    leg(end+1) = "q_{1,est}";

    if isscalar(plotData.q1_d)
        h(end+1) = plot(ax,[t(1) t(end)],[plotData.q1_d plotData.q1_d], ...
            'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_q1_d);
    else
        h(end+1) = plot(ax,t,plotData.q1_d,'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_q1_d);
    end
    leg(end+1) = "q_{1,d}";

    h(end+1) = plot(ax,t,plotData.q2,'LineStyle',ls,'Marker',mk,'LineWidth',lw_state,'Color',c_q2);
    leg(end+1) = "q_2";

    h(end+1) = plot(ax,t,plotData.q2_est,'LineStyle',ls,'Marker',mk,'LineWidth',lw_est,'Color',c_q2_est);
    leg(end+1) = "q_{2,est}";

    if isscalar(plotData.q2_d)
        h(end+1) = plot(ax,[t(1) t(end)],[plotData.q2_d plotData.q2_d], ...
            'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_q2_d);
    else
        h(end+1) = plot(ax,t,plotData.q2_d,'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_q2_d);
    end
    leg(end+1) = "q_{2,d}";

    ylabel(ax,'Angle [rad]');

    % -------------------------
    % RIGHT AXIS: velocities
    % -------------------------
    yyaxis(ax,'right');

    h(end+1) = plot(ax,t,plotData.dot_q1,'LineStyle',ls,'Marker',mk,'LineWidth',lw_state,'Color',c_dq1);
    leg(end+1) = "dot{q}_1";

    h(end+1) = plot(ax,t,plotData.dot_q1_est,'LineStyle',ls,'Marker',mk,'LineWidth',lw_est,'Color',c_dq1_est);
    leg(end+1) = "dot{q}_{1,est}";

    if isscalar(plotData.dot_q1_d)
        h(end+1) = plot(ax,[t(1) t(end)],[plotData.dot_q1_d plotData.dot_q1_d], ...
            'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_dq1_d);
    else
        h(end+1) = plot(ax,t,plotData.dot_q1_d,'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_dq1_d);
    end
    leg(end+1) = "dot{q}_{1,d}";

    h(end+1) = plot(ax,t,plotData.dot_q2,'LineStyle',ls,'Marker',mk,'LineWidth',lw_state,'Color',c_dq2);
    leg(end+1) = "dot{q}_2";

    h(end+1) = plot(ax,t,plotData.dot_q2_est,'LineStyle',ls,'Marker',mk,'LineWidth',lw_est,'Color',c_dq2_est);
    leg(end+1) = "dot{q}_{2,est}";

    if isscalar(plotData.dot_q2_d)
        h(end+1) = plot(ax,[t(1) t(end)],[plotData.dot_q2_d plotData.dot_q2_d], ...
            'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_dq2_d);
    else
        h(end+1) = plot(ax,t,plotData.dot_q2_d,'LineStyle',ls,'Marker',mk,'LineWidth',lw_des,'Color',c_dq2_d);
    end
    leg(end+1) = "dot{q}_{2,d}";

    ylabel(ax,'Angular velocity [rad/s]');

    % -------------------------
    % Legend with toggle support
    % -------------------------
    xlabel(ax,'Time [s]');
    title(ax,'States, Estimated States and Desired States');

    lg = legend(ax, h, leg, 'Location','best');
    set(lg,'ItemHitFcn',@legendItemToggle);

    grid(ax,'on');
    hold(ax,'off');
end
function legendItemToggle(~, event)
    obj = event.Peer;
    if strcmp(obj.Visible,'on')
        obj.Visible = 'off';
    else
        obj.Visible = 'on';
    end
end
