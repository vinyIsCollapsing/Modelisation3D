clc; clear;

disp('--- Testando Cinemática Direta ---');
q_neutra = zeros(1,7);
[T07, T_all] = MGD(q_neutra);
disp('Pose do efetuador (T07):');
disp(T07);

disp('--- Visualizando Robô ---');
plot_robot(q_neutra); % Descomente para ver o robô parado

disp('--- Gerando Animação (GIF) ---');
q_start = zeros(1,7);
q_end = [pi/4, pi/6, -pi/4, 0, pi/3, -pi/4, 0];

% Gera o arquivo 'lbr4_demo.gif' com 50 frames
animate_robot(q_start, q_end, 'lbr4_demo.gif', 50);