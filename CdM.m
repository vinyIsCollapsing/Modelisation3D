all clc;

alpha = [0.0, pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2];
d = zeros(7);
r = [0.3105, 0.0, 0.4, 0.0, 0.39, 0.0, 0.078];
theta = zeros(7);
m = [2.7, 2.7, 2.7, 2.7, 2.7, 2.7, 0.3];
c = zeros(4, 7);
c(2,:) = [-8.70e-3, 8.7e-3, 8.7e-3, -8.70e-3, -8.2e-3, -7.6e-3, 0.0];
c(3,:) = [-1.461e-2, -1.461e-2, -1.461e-2, -1.461e-2, -3.48e-2, 1.363e-3, 0.0];
c(4,:) = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
M = 0;
for i = numel(m)
    M = M + m(i);
end

T = zeros(4,4,length(d));
CenterMass = zeros(4,1);
for i = 1 : length(d)
    T(:,:,i) = [cos(theta(i)) -sin(theta(i)) 0 d(i);
    cos(alpha(i))*sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i)) -r(i)*sin(alpha(i));
    sin(alpha(i))*sin(theta(i)) sin(alpha(i))*cos(theta(i)) cos(alpha(i)) r(i)*cos(alpha(i));
    0 0 0 1];
    
    % Correction pour sauvegarder les operations matricielles
    if i >= 2
        T(:,:,i) = T(:,:,i - 1) * T(:,:,i);
    end

end

for i = 1 : length(d)


    CenterMass = (m(i)/M)*(T(i))*c(1:end,i) + CenterMass;
end
