clc;
clear;

% Paramètres initiaux
alpha = [0.0, -pi/2, 0.0, -pi/2, pi/2, -pi/2];
d = [0.0, 0.0, 265.69, 30.0, 0.0, 0.0];
theta = 0.0;
thetaEnsemble = [theta, theta - 1.4576453, theta - 0.898549163, theta, theta, theta, 180];
r = [159.0, 0.0, 0.0, 258.0, 0.0, 0.0, 123.0];

% Matrices pour la Jacobienne
J = zeros(6, 6, length(d)); % Matrice Jacobienne
T = zeros(4, 4, length(d)); % Matrices de transformation homogènes

% Variables pour les axes et les orientations
axes = zeros(3, 1, length(d) + 1); % Positions des axes
orientation = zeros(3, 3, length(d) + 1); % Orientations des axes

% Initialisation de la première matrice de transformation à l'identité
T(:,:,1) = eye(4);

% Boucle pour calculer les matrices de transformation homogènes et la Jacobienne
for i = 1:length(d)
    % Matrice de transformation pour chaque articulation
    T(:,:,i+1) = [cos(thetaEnsemble(i)), -sin(thetaEnsemble(i)), 0, r(i);
                  cos(alpha(i))*sin(thetaEnsemble(i)), cos(alpha(i))*cos(thetaEnsemble(i)), -sin(alpha(i)), -d(i)*sin(alpha(i));
                  sin(alpha(i))*sin(thetaEnsemble(i)), sin(alpha(i))*cos(thetaEnsemble(i)), cos(alpha(i)), d(i)*cos(alpha(i));
                  0, 0, 0, 1];
    
    % Multiplier par la transformation précédente (cumulative)
    T(:,:,i+1) = T(:,:,i) * T(:,:,i+1);
    
    % Extraire la position (colonne 4) et l'orientation (3 premières colonnes) 
    axes(:, :, i+1) = T(1:3, 4, i+1); % Position
    orientation(:, :, i+1) = T(1:3, 1:3, i+1); % Matrice de rotation
    
    % Stockage de R (rotation) et P (position)
    R = T(1:3, 1:3, i+1); % Matrice de rotation
    P = T(1:3, 4, i+1);   % Vecteur de position
    a = R(:, 3);          % Axe z de la matrice de rotation

    % Vous pouvez utiliser ces valeurs pour calculer la Jacobienne ici si nécessaire
end

% Afficher les résultats pour vérification
disp('Matrices de transformation homogènes :');
disp(T);

disp('Axes (positions) :');
disp(axes);

disp('Orientations (rotations) :');
disp(orientation);
