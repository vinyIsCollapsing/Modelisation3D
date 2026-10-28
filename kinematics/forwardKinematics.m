function [T07, T_all] = MGD(q)
%   MGD  Modelo Geometrico Direto do KUKA LBR4+ (7 eixos).
%   Convenção DH: modificada (Craig).
%   a = angulo de torcao (alpha de Craig)
%   d = comprimento do elo em X (a de Craig)
%   r = offset ao longo de Z (d de Craig)

    % ---------- Parametros DH fixos (LBR4+) ----------
    a = [0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2];
    d = [0, 0, 0, 0, 0, 0, 0];
    r = [310.4, 0, 400.1, 0, 390.0, 0, 78.0];

    % ---------- Offsets das juntas ----------
    varQ = zeros(7,1);
    q = q(:) + varQ;

    % ---------- Acumulacao ----------
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