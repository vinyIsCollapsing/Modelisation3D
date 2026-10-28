addpath('kinematics');
addpath('visualization');
addpath('validation');

clc; clear;

disp('--- Testing Forward Kinematics ---');
q_neutra = zeros(1,7);
[T07, T_all] = forwardKinematics(q_neutra);
disp('Effector pose (T07):');
disp(T07);

disp('--- Visualizing the Robot ---');
plotRobot(q_neutra); % Uncomment to see the robot at rest

disp('--- Generating Animation (GIF) ---');
q_start = zeros(1,7);
q_end = [pi/4, pi/6, -pi/4, 0, pi/3, -pi/4, 0];

% Generates the file 'lbr4_demo.gif' with 50 frames
animateRobot(q_start, q_end, 'docs/lbr4_demo.gif', 50);