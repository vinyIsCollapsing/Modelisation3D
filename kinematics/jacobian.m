function J = jacobian(q, p0)
%   jacobian  Geometric Jacobian of the KUKA LBR4+ (7 axes).

    a = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];
    d = [0, 0, 0, 0, 0, 0, 0];
    r = [310.4, 0, 400.1, 0, 390.0, 0, 78.0];

    varQ = zeros(7,1);
    q = q(:) + varQ;
    p0 = p0(:);

    Tp = eye(4);
    J  = zeros(6, 7);

    for i = 1:7
        Ti = [ cos(q(i)),            -sin(q(i)),              0,          d(i);
               cos(a(i))*sin(q(i)),   cos(a(i))*cos(q(i)),   -sin(a(i)), -r(i)*sin(a(i));
               sin(a(i))*sin(q(i)),   sin(a(i))*cos(q(i)),    cos(a(i)),  r(i)*cos(a(i));
               0,                     0,                      0,          1 ];

        Tp = Tp * Ti;
        z_i = Tp(1:3, 3);
        o_i = Tp(1:3, 4);
        l_i = p0 - o_i;

        J(1:3, i) = cross(z_i, l_i);
        J(4:6, i) = z_i;
    end
end