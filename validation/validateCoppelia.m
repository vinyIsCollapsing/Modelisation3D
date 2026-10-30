function validateCoppelia()
    addpath('zmqRemoteApi/clients/octave');
    addpath('kinematics');
    
    % --- 1. Connect to CoppeliaSim ---
    client = RemoteAPIClient();
    sim = client.require('sim');
    
    % --- 2. Joint names ---
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
    tip_handle = sim.getObject('/LBR4p/connection');

    % --- 3. Define the test pose ---
    % q = [0, 0, 0, 0, 0, 0, 0];
    q = [pi/4, pi/6, -pi/4, 0, pi/3, -pi/4, 0];
    
    for i = 1:7
        sim.setJointPosition(joint_handles(i), q(i));
    end
    
    pause(0.1); 
    
    % --- 4. Read the actual position from CoppeliaSim ---
    pos_coppelia_raw = sim.getObjectPosition(tip_handle, sim.handle_world);
    
    if iscell(pos_coppelia_raw)
        pos_coppelia = cell2mat(pos_coppelia_raw);
    else
        pos_coppelia = pos_coppelia_raw;
    end
    pos_coppelia = pos_coppelia(:);
    
    % --- 5. Calculate using forward kinematics ---
    [T07, ~] = forwardKinematics(q); 
    pos_octave_mm = T07(1:3, 4);
    pos_octave_m = pos_octave_mm / 1000.0; 
    
    % --- 6. Base alignment matrix (Empirical finding 'validation/alignmentMatrix.m') ---
    theta_base = 0; 
    Rz = [cos(theta_base), -sin(theta_base), 0;
          sin(theta_base),  cos(theta_base), 0;
          0,               0,                1];
          
    % Base translation (XY offset calculated from the difference between the centers)
    X_offset = -0.25;
    Y_offset =  0.25;
    Z_offset =  0.00;
    
    % Apply the base to the Octave position
    p_octave_adjusted = Rz * pos_octave_m + [X_offset; Y_offset; Z_offset];
    
    % --- 7. Compare results ---
    fprintf('\n=== Validation Results ===\n');
    fprintf('Position (CoppeliaSim) [m]: X=%.4f, Y=%.4f, Z=%.4f\n', ...
        double(pos_coppelia(1)), double(pos_coppelia(2)), double(pos_coppelia(3)));
        
    fprintf('Position (Octave FK [adjusted])   [m]: X=%.4f, Y=%.4f, Z=%.4f\n', ...
        double(p_octave_adjusted(1)), double(p_octave_adjusted(2)), double(p_octave_adjusted(3)));
    
    erro_final = norm(pos_coppelia - p_octave_adjusted);
    fprintf('Position error: %.4f meters (%.2f mm)\n', double(erro_final), double(erro_final)*1000);
    
    if erro_final < 0.010 
        fprintf('SUCCESS! The mathematical model has been validated.\n');
    else
        fprintf('Still a discrepancy.\n');
    end
end

% First test
% Using "/LBR4p/connection" as the reference point (end effector).
% Sending joint angles to CoppeliaSim...
% === Validation Results ===
% Position (CoppeliaSim) [m]: X=-0.1356, Y=0.3644, Z=1.0454
% Position (Octave FK)   [m]: X=0.2763, Y=0.2561, Z=1.0690
% Position error: 0.4266 meters (426.59 mm)

% Second test
% === Validation Results ===
% Position (CoppeliaSim) [m]: X=-0.1356, Y=0.3644, Z=1.0454
% Position (Octave FK [adjusted])   [m]: X=-0.1315, Y=0.3531, Z=1.0690
% Position error: 0.0265 meters (26.54 mm)

% Third test
% === Validation Results ===
% Position (CoppeliaSim) [m]: X=-0.5263, Y=-0.0061, Z=1.0691
% Position (Octave FK [adjusted])   [m]: X=-0.5263, Y=-0.0061, Z=1.0690
% Position error: 0.0000 meters (0.04 mm)
% SUCCESS! The mathematical model has been validated.