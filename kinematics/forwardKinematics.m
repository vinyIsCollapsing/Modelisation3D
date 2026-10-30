function [T07, T_all] = forwardKinematics(q)
%   forwardKinematics: Direct Geometric Model of the KUKA LBR4+ (7 axes)
%   DH convention: modified (Craig)
%   a = torque angle (Craig alpha)
%   d = link length in the X-direction (Craig a)
%   r = offset along the Z-axis (Craig’s d)

    % ---------- Fixed DH parameters (LBR4+) ----------
    a = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];
    d = [0, 0, 0, 0, 0, 0, 0];
    r = [310.4, 0, 400.1, 0, 390.0, 0, 78.0];

    % ---------- Joint offsets ----------
    varQ = zeros(7,1);
    q = q(:) + varQ;
    
    % Joints 2, 4, and 6 on the CoppeliaSim rotate in the opposite direction from our DH model.
    q(2) = -q(2);
    q(4) = -q(4);
    q(6) = -q(6);

    % ---------- Accumulation ----------
    T07   = eye(4);
    T_all = zeros(4, 4, 7);

    for i = 1:7
        Ti = [ cos(q(i)),            -sin(q(i)),              0,          d(i);
               cos(a(i))*sin(q(i)),   cos(a(i))*cos(q(i)),   -sin(a(i)), -r(i)*sin(a(i));
               sin(a(i))*sin(q(i)),   sin(a(i))*cos(q(i)),    cos(a(i)),  r(i)*cos(a(i));
               0,                     0,                      0,          1 ];

        T07 = T07 * Ti;
        T_all(:,:,i) = T07;
    end
end