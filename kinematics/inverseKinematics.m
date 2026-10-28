function q = ComCinematicInverse(T_target, q0, max_iter, tol)
%   COMCINEMAICINVVERSE  Cinematica inversa numerica do LBR4+ (7 eixos).

    if nargin < 3, max_iter = 100; end
    if nargin < 4, tol = 1e-3; end

    q = q0(:);
    p_target = T_target(1:3, 4);

    for iter = 1:max_iter
        [T_cur, ~] = MGD(q);
        p_cur = T_cur(1:3, 4);

        e = p_target - p_cur;

        if norm(e) < tol
            fprintf('Convergiu em %d iterações (erro = %.2e)\n', iter, norm(e));
            return;
        end

        J = MatriceJacobienne(q, p_cur);
        
        % Pseudoinversa para matriz 6x7
        dq = pinv(J) * [e; 0; 0; 0]; 
        
        q = q + 0.5 * dq; % Passo com amortecimento
    end

    warning('IK não convergiu em %d iterações (erro = %.2e)', max_iter, norm(e));
end