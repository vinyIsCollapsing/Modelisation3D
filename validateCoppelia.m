function validateCoppelia()
    addpath('zmqRemoteApi/clients/octave');
    addpath('kinematics');

    % --- 1. Connect to CoppeliaSim ---
    client = RemoteAPIClient();
    sim = client.require('sim');

    % --- 2. Joint names ---
    joint_names = {'/LBR4p/joint', '/LBR4p/link/joint', '/LBR4p/joint/link/joint', ...
                   '/LBR4p/joint/link/joint/link/joint', '/LBR4p/joint/link/joint/link/joint/link/joint', ...
                   '/LBR4p/joint/link/joint/link/joint/link/joint/link/joint', ...
                   '/LBR4p/joint/link/joint/link/joint/link/joint/link/joint/link/joint'};

    joint_handles = zeros(1, 7);
    for i = 1:7
        joint_handles(i) = sim.getObject(joint_names{i});
    end

    % --- 3. Find the "Tip" by traversing the robot tree ---
    tip_name = '/LBR4p/connection';
    tip_handle = sim.getObject(tip_name);
    fprintf('Using "%s" as the reference point (end effector).\n', tip_name);

    % --- 4. Define the test pose ---
    q = [pi/4, pi/6, -pi/4, 0, pi/3, -pi/4, 0];

    fprintf('\nSending joint angles to CoppeliaSim...\n');
    for i = 1:7
        sim.setJointPosition(joint_handles(i), q(i));
    end

    pause(0.1);

    % --- 5. Read the actual position from CoppeliaSim ---
    pos_coppelia_raw = sim.getObjectPosition(tip_handle, sim.handle_world);

    if iscell(pos_coppelia_raw)
        pos_coppelia = cell2mat(pos_coppelia_raw);
    else
        pos_coppelia = pos_coppelia_raw;
    end
    pos_coppelia = pos_coppelia(:);

    % --- 6. Calculate using forward kinematics ---
    [T07, ~] = forwardKinematics(q);
    pos_octave_mm = T07(1:3, 4);
    pos_octave_m = pos_octave_mm / 1000.0;

    % --- 7. Compare results ---
    fprintf('\n=== Validation Results ===\n');
    fprintf('Position (CoppeliaSim) [m]: X=%.4f, Y=%.4f, Z=%.4f\n', ...
        double(pos_coppelia(1)), double(pos_coppelia(2)), double(pos_coppelia(3)));

    fprintf('Position (Octave FK)   [m]: X=%.4f, Y=%.4f, Z=%.4f\n', ...
        double(pos_octave_m(1)), double(pos_octave_m(2)), double(pos_octave_m(3)));

    error = norm(pos_coppelia - pos_octave_m);
    fprintf('Position error: %.4f meters (%.2f mm)\n', double(error), double(error)*1000);
end

% First test
% Using "/LBR4p/connection" as the reference point (end effector).
% Sending joint angles to CoppeliaSim...
% === Validation Results ===
% Position (CoppeliaSim) [m]: X=-0.1356, Y=0.3644, Z=1.0454
% Position (Octave FK)   [m]: X=0.2763, Y=0.2561, Z=1.0690
% Position error: 0.4266 meters (426.59 mm)
