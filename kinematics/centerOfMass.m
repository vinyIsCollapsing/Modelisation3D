clc; clear;

%   DH Parameters (LBR4+)
alpha = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];
d     = [0, 0, 0, 0, 0, 0, 0];
r     = [310.4, 0, 400.1, 0, 390.0, 0, 78.0];
theta = zeros(1, 7); % Neutral pose

%   Masses and centers of mass (approximate values)
m = [2.0, 2.0, 2.0, 1.5, 1.5, 1.0, 0.5];   % kg
c = zeros(4, 7);
c(1,:) = zeros(1,7);
c(2,:) = zeros(1,7);
c(3,:) = [ -155, 0, -200, 0, -195, 0, -39 ]; % CM at the center of each link
c(4,:) = ones(1, 7);

M = sum(m);
T = zeros(4, 4, 7);

for i = 1:7
    Ti = [ cos(theta(i)),            -sin(theta(i)),              0,          d(i);
           cos(alpha(i))*sin(theta(i)), cos(alpha(i))*cos(theta(i)), -sin(alpha(i)), -r(i)*sin(alpha(i));
           sin(alpha(i))*sin(theta(i)), sin(alpha(i))*cos(theta(i)),  cos(alpha(i)),  r(i)*cos(alpha(i));
           0,                           0,                            0,          1 ];

    if i == 1
        T(:,:,i) = Ti;
    else
        T(:,:,i) = T(:,:,i-1) * Ti;
    end
end

CenterMass = zeros(3, 1);
for i = 1:7
    cm_i = T(:,:,i) * c(:,i);
    CenterMass = CenterMass + (m(i)/M) * cm_i(1:3);
end

disp('Global center of mass (mm):');
disp(CenterMass);