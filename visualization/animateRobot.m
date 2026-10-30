function animateRobot(q_start, q_end, filename, num_frames)
%   animateRobot Creates an animated GIF of the KUKA LBR4+ moving from q_start to q_end.

    if nargin < 3
        filename = 'lbr4_anim.gif';
    end
    if nargin < 4
        num_frames = 40;
    end

    [folder, ~, ~] = fileparts(filename);
    if ~isempty(folder) && ~exist(folder, 'dir')
        mkdir(folder);
    end

    q_start = q_start(:);
    q_end   = q_end(:);

    fig = figure('Visible', 'off', 'Position', [100 100 1200 500]);
    
    ax_3d  = subplot(1, 2, 1, 'Parent', fig);
    ax_top = subplot(1, 2, 2, 'Parent', fig);

    lims = [-800 800 -800 800 0 1200];

    fprintf('Generating %d frames...\n', num_frames);

    for k = 1:num_frames
        t = (k - 1) / (num_frames - 1);
        t_smooth = 3*t^2 - 2*t^3; % Smoothstep

        q_cur = (1 - t_smooth) * q_start + t_smooth * q_end;

        cla(ax_3d);
        cla(ax_top);
        
        % --- Screen 1: 3D View ---
        plotRobot(q_cur, 'Axes', ax_3d, 'Limits', lims, 'View', 3);
        title(ax_3d, '3D Perspective');
        
        % --- Screen 2: Top View ---
        plotRobot(q_cur, 'Axes', ax_top, 'Limits', lims, 'View', 2);
        title(ax_top, 'Top View (XY Plane)');
        drawnow;

        frame_name = sprintf('docs/frame_%03d.png', k);
        print(fig, frame_name, '-dpng', '-r100');

        img = imread(frame_name);

        if k == 1
            imwrite(img, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
        else
            imwrite(img, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        end

        delete(frame_name);
    end

    fprintf('GIF successfully saved: %s\n', filename);
    close(fig);
end