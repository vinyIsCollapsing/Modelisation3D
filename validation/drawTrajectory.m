function drawTrajectory()
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
    
    % --- 3. Fetch Limits automatically ---
    [q_min, q_max] = getJointLimits();
    
    % --- 4. Define the Circle Trajectory ---
    center = [400; 0; 600]; % X Y Z
    radius = 150; 
    num_points = 60;
    
    % Create all the points
    theta = linspace(0, 2*pi, num_points);
    traj_x = center(1) + radius * cos(theta);
    traj_y = center(2) + radius * sin(theta);
    traj_z = center(3) * ones(1, num_points); % Keep same height
    
    fprintf('Starting simulation...\n');
    sim.startSimulation();
    
    q_current = zeros(7,1);
    
    fprintf('Drawing circle with %d points...\n', num_points);
    
    for i = 1:num_points
        target_pos = [traj_x(i); traj_y(i); traj_z(i)];
        
        % Resolve IK
        [q_sol, success] = inverseKinematics(target_pos, q_current, q_min, q_max, 500, 0.5);
        
        if ~success
            fprintf('  Point %d: IK failed. Skipping.\n', i);
            continue;
        end
        
        % Send to CoppeliaSim
        for j = 1:7
            try
                sim.setJointTargetPosition(joint_handles(j), q_sol(j));
            catch
                sim.setJointPosition(joint_handles(j), q_sol(j));
            end
        end
        
        % Update q_current to next point
        q_current = q_sol;

        if mod(i, 10) == 0
            fprintf('  %d/%d points drawn.\n', i, num_points);
        end
    end
    
    fprintf('Trajectory complete! Stopping simulation.\n');
    sim.stopSimulation();
end