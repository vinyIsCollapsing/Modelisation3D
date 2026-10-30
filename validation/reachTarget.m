function reachTarget()
    % Add paths
    addpath('kinematics');
    addpath('zmqRemoteApi/clients/octave');
    
    % --- 1. Connect to CoppeliaSim ---
    client = RemoteAPIClient();
    sim = client.require('sim');
    
    % --- 2. Get Joint Handles ---
    joint_names = { '/LBR4p/joint', ...
                    '/LBR4p/link/joint', ...
                    '/LBR4p/link/joint/link/joint', ...
                    '/LBR4p/link/joint/link/joint/link/joint', ...
                    '/LBR4p/link/joint/link/joint/link/joint/link/joint', ...
                    '/LBR4p/link/joint/link/joint/link/joint/link/joint/link/joint', ...
                    '/LBR4p/link/joint/link/joint/link/joint/link/joint/link/joint/link/joint'};
               
    joint_handles = zeros(1, 7);
    for i = 1:7
        joint_handles(i) = sim.getObject(joint_names{i});
    end
    
    % --- 3. Define the Target ---
    target_pos = [200; 100; 600]; 
    fprintf('Target Position: [%d, %d, %d] mm\n', target_pos(1), target_pos(2), target_pos(3));
    
    % --- 4. Define Limits Manually (to avoid API bugs) ---
    q_min = deg2rad([-170; -120; -170; -120; -170; -120; -175]);
    q_max = deg2rad([ 170;  120;  170;  120;  170;  120;  175]);
    
    % --- 5. Solve Inverse Kinematics ---
    q0 = zeros(7,1); 
    fprintf('Solving Inverse Kinematics...\n');
    [q_sol, success] = inverseKinematics(target_pos, q0, q_min, q_max);
    
    if ~success
        warning('IK did not converge.');
        return;
    end
    
    % Print the solution to see if it makes sense
    fprintf('\nCalculated Joint Angles (degrees):\n');
    disp(rad2deg(q_sol)');
    
    % --- 6. Start Simulation and Send Angles ---
    fprintf('Starting simulation and sending angles...\n');
    
    % Para modelos dinamicos no Coppelia, o ideal eh setar o alvo do motor
    % e depois rodar a simulacao por alguns passos.
    for i = 1:7
        % Tenta usar setJointTargetPosition (para juntas dinamicas)
        try
            sim.setJointTargetPosition(joint_handles(i), q_sol(i));
        catch
            % Se falhar, usa setJointPosition (para juntas cinematicas)
            sim.setJointPosition(joint_handles(i), q_sol(i));
        end
    end
    
    % Roda a simulacao por um tempinho para a fisica atualizar
    sim.startSimulation();
    pause(0.5); 
    
    % --- DEBUG: Ler a posicao da junta logo apos o comando ---
    pos_junta1 = sim.getJointPosition(joint_handles(1));
    fprintf('Debug: Junta 1 enviada = %.2f deg, Junta 1 lida = %.2f deg\n', ...
        rad2deg(q_sol(1)), rad2deg(pos_junta1));
    pos_junta2 = sim.getJointPosition(joint_handles(2));
    fprintf('Debug: Junta 2 enviada = %.2f deg, Junta 2 lida = %.2f deg\n', ...
        rad2deg(q_sol(2)), rad2deg(pos_junta2));
    pos_junta3 = sim.getJointPosition(joint_handles(3));
    fprintf('Debug: Junta 3 enviada = %.2f deg, Junta 3 lida = %.2f deg\n', ...
        rad2deg(q_sol(3)), rad2deg(pos_junta3));
    pos_junta4 = sim.getJointPosition(joint_handles(4));
    fprintf('Debug: Junta 4 enviada = %.2f deg, Junta 4 lida = %.2f deg\n', ...
        rad2deg(q_sol(4)), rad2deg(pos_junta4));
    pos_junta5 = sim.getJointPosition(joint_handles(5));
    fprintf('Debug: Junta 5 enviada = %.2f deg, Junta 5 lida = %.2f deg\n', ...
        rad2deg(q_sol(5)), rad2deg(pos_junta5));
    pos_junta6 = sim.getJointPosition(joint_handles(6));
    fprintf('Debug: Junta 6 enviada = %.2f deg, Junta 6 lida = %.2f deg\n', ...
        rad2deg(q_sol(6)), rad2deg(pos_junta6));
    pos_junta7 = sim.getJointPosition(joint_handles(7));
    fprintf('Debug: Junta 7 enviada = %.2f deg, Junta 7 lida = %.2f deg\n', ...
        rad2deg(q_sol(7)), rad2deg(pos_junta7));
    
    % --- 7. Read World Position ---
    tip_handle = sim.getObject('/LBR4p/connection');
    pos_coppelia_raw = sim.getObjectPosition(tip_handle, sim.handle_world);
    
    if iscell(pos_coppelia_raw)
        pos_coppelia = cell2mat(pos_coppelia_raw);
    else
        pos_coppelia = pos_coppelia_raw;
    end
    pos_coppelia = pos_coppelia(:) * 1000; % Convert to mm
    
    % --- 8. Stop Simulation ---
    sim.stopSimulation();
    
    % --- 9. Calculate and Print Results ---
    fprintf('\nCoppeliaSim World Pos [mm]: [%.1f, %.1f, %.1f]\n', pos_coppelia(1), pos_coppelia(2), pos_coppelia(3));
    
    % Apply the 67.6 degree base rotation to compare
    theta_base = pi / 180;
    Rz = [cos(theta_base), -sin(theta_base), 0;
          sin(theta_base),  cos(theta_base), 0;
          0,               0,                1];
    pos_coppelia_aligned = Rz' * pos_coppelia; 
    
    fprintf('CoppeliaSim Aligned Pos [mm]: [%.1f, %.1f, %.1f]\n', pos_coppelia_aligned(1), pos_coppelia_aligned(2), pos_coppelia_aligned(3));
    
    error_mm = norm(target_pos - pos_coppelia_aligned);
    fprintf('Final Error: %.2f mm\n', error_mm);
end