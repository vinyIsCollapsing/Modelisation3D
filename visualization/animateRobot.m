function animate_robot(q_start, q_end, filename, num_frames)
%   ANIMATE_ROBOT Cria um GIF animado do KUKA LBR4+ movendo-se de q_start a q_end.

    if nargin < 3
        filename = 'lbr4_anim.gif';
    end
    if nargin < 4
        num_frames = 40;
    end

    q_start = q_start(:);
    q_end   = q_end(:);

    fig = figure('Visible', 'off', 'Position', [100 100 800 600]);
    ax = axes('Parent', fig);
    
    lims = [-800 800 -800 800 0 1200];
    axis(ax, lims);
    
    fprintf('Gerando %d frames...\n', num_frames);
    
    for k = 1:num_frames
        t = (k - 1) / (num_frames - 1);
        t_smooth = 3*t^2 - 2*t^3; % Smoothstep
        
        q_cur = (1 - t_smooth) * q_start + t_smooth * q_end;
        
        cla(ax);
        plot_robot(q_cur, 'Axes', ax, 'Limits', lims);
        drawnow;
        
        frame_name = sprintf('frame_%03d.png', k);
        print(fig, frame_name, '-dpng', '-r100');
        
        img = imread(frame_name);
        
        if k == 1
            imwrite(img, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
        else
            imwrite(img, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        end
        
        delete(frame_name);
    end
    
    fprintf('GIF salvo com sucesso: %s\n', filename);
    close(fig);
end