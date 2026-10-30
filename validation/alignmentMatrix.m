addpath('kinematics');

% Data
q = [0, 0, 0, 0, 0, 0, 0];
[T07, ~] = forwardKinematics(q);
p_octave = T07(1:3, 4) / 1000; 
p_coppelia = [-0.2500; 0.2500; 1.1785];

best_error = inf;
best_angle = 0;

for deg = 0:0.1:360
    theta = deg * pi / 180;
    R_z = [cos(theta), -sin(theta), 0;
           sin(theta),  cos(theta), 0;
           0,           0,          1];
           
    p_rot = R_z * p_octave;
    
    % Calculates the distance between two points in the XY plane (ignoring Z for now)
    erro_xy = norm(p_coppelia(1:2) - p_rot(1:2));
    
    if erro_xy < best_error
        best_error = erro_xy;
        best_angle = deg;
    end
end

fprintf('Best Z-axis rotation angle: %d degrees\n', best_angle);
fprintf('Minimum error in the XY plane: %.2f mm\n', best_error * 1000);

% Checks the error on the Z-axis for this best angle
theta = best_angle * pi / 180;
R_z = [cos(theta), -sin(theta), 0;
       sin(theta),  cos(theta), 0;
       0,           0,          1];
p_final = R_z * p_octave;
error_z = abs(p_coppelia(3) - p_final(3)) * 1000;

fprintf('Error on the Z-axis: %.2f mm\n', error_z);