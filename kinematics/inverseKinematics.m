function q = inverseKinematics(T_target, q0, max_iter, tol)
%   inverseKinematics  Numerical inverse kinematics for the LBR4+ (7 axes).

    if nargin < 3, max_iter = 100; end
    if nargin < 4, tol = 1e-3; end

    q = q0(:);
    p_target = T_target(1:3, 4);

    for iter = 1:max_iter
        [T_cur, ~] = MGD(q);
        p_cur = T_cur(1:3, 4);

        e = p_target - p_cur;

        if norm(e) < tol
            fprintf('Converged in %d iterations (error = %.2e)\n', iter, norm(e));
            return;
        end

        J = MatriceJacobienne(q, p_cur);

        % Pseudoinverse for a 6x7 matrix
        dq = pinv(J) * [e; 0; 0; 0];

        q = q + 0.5 * dq; % Step with damping
    end

    warning('IK did not converge in %d iterations (error = %.2e)', max_iter, norm(e));
end
