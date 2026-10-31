function reachTarget()
    addpath('kinematics');
    addpath('zmqRemoteApi/clients/octave');
    
    % --- 1. Connect to CoppeliaSim ---
    client = RemoteAPIClient();
    sim = client.require('sim');
    
    % --- 2. Joint names (using the ones that worked in validateCoppelia) ---
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
    target_pos = [300; 200; 600]; 
    fprintf('Target Position: [%d, %d, %d] mm\n', target_pos(1), target_pos(2), target_pos(3));
    
    % --- 4. Define Limits Automatically ---
    [q_min, q_max] = getJointLimits();
    
    % --- 5. Solve Inverse Kinematics ---
    q0 = zeros(7,1); 
    fprintf('Solving Inverse Kinematics...\n');
    
    [q_sol, success] = inverseKinematics(target_pos, q0, q_min, q_max);
    
    if ~success
        warning('IK did not converge.');
        return;
    end
    
    fprintf('\nCalculated Joint Angles (degrees):\n');
    disp(rad2deg(q_sol)');
    
    % --- 6. Start Simulation and Send Angles ---
    fprintf('Starting simulation and sending angles...\n');
    
    for i = 1:7
        try
            sim.setJointTargetPosition(joint_handles(i), q_sol(i));
        catch
            sim.setJointPosition(joint_handles(i), q_sol(i));
        end
    end
    
    sim.startSimulation();
    pause(1.0);
    
    % --- 7. Read World Position ---
    tip_handle = sim.getObject('/LBR4p/connection');
    pos_coppelia_raw = sim.getObjectPosition(tip_handle, sim.handle_world);
    
    if iscell(pos_coppelia_raw)
        pos_coppelia = cell2mat(pos_coppelia_raw);
    else
        pos_coppelia = pos_coppelia_raw;
    end
    pos_coppelia = pos_coppelia(:) * 1000; % Convert to mm
    
    sim.stopSimulation();
    
    % --- 8. Calculate and Print Results ---
    % Applies the exact correction discovered in validateCoppelia
    X_offset =  250; % Convert to mm (-0.25 m)
    Y_offset =  -250; % Convert to mm ( 0.25 m)
    Z_offset =  0;
    
    pos_coppelia_aligned = pos_coppelia + [X_offset; Y_offset; Z_offset];
    
    fprintf('\nCoppeliaSim World Pos [mm]: [%.1f, %.1f, %.1f]\n', pos_coppelia(1), pos_coppelia(2), pos_coppelia(3));
    fprintf('CoppeliaSim Aligned Pos [mm]: [%.1f, %.1f, %.1f]\n', pos_coppelia_aligned(1), pos_coppelia_aligned(2), pos_coppelia_aligned(3));
    
    error_mm = norm(target_pos - pos_coppelia_aligned);
    fprintf('Final Error: %.2f mm\n', error_mm);
end