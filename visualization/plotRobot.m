function plotRobot(q, varargin)
%   plotRobot  Plots the KUKA LBR4+ (7 axes) in 3D for a pose q.

    p = inputParser;
    addParameter(p, 'Axes',     []);
    addParameter(p, 'Color',    [0.2 0.4 0.9]);
    addParameter(p, 'Marker',   'o');
    addParameter(p, 'ShowBase', true);
    addParameter(p, 'ShowEE',   true);
    addParameter(p, 'Limits',   []);
    addParameter(p, 'View',     3);
    parse(p, varargin{:});
    opt = p.Results;

    [~, T_all] = forwardKinematics(q);
    P = squeeze(T_all(1:3, 4, :));        
    P = [ [0;0;0], P ];                   

    if isempty(opt.Axes)
        figure;
        ax = axes;
    else
        ax = opt.Axes;
        axes(ax);
    end
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, opt.View);
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    title(ax, 'KUKA LBR4+ — direct kinematics (DH, 7 axes)');

    plot3(ax, P(1,:), P(2,:), P(3,:), '-', 'Color', opt.Color, 'LineWidth', 3);
    plot3(ax, P(1,2:end), P(2,2:end), P(3,2:end), opt.Marker, 'Color', opt.Color, ...
          'MarkerFaceColor', opt.Color, 'MarkerSize', 8);

    if opt.ShowBase
        b = 30;
        X = [-b b b -b -b]; Y = [-b -b b b -b]; Z = zeros(1,5);
        plot3(ax, X, Y, Z, 'k-', 'LineWidth', 1);
    end

    if opt.ShowEE
        ee = P(:, end);
        plot3(ax, ee(1), ee(2), ee(3), 'p', 'Color', [0.9 0.1 0.1], ...
              'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerSize', 14);
    end

    if ~isempty(opt.Limits)
        axis(ax, opt.Limits);
    else
        span = max(P, [], 2) - min(P, [], 2);
        center = mean(P, 2);
        half = max(max(span) * 0.7, 300);
        axis(ax, [center(1)-half, center(1)+half, center(2)-half, center(2)+half, center(3)-half, center(3)+half]);
    end

    hold(ax, 'off');
    drawnow;
end