function [q, success] = inverseKinematics(target_pos, q0, q_min, q_max, max_iter, tol)
%   inverseKinematics  Numerical inverse kinematics for the LBR4+ (7 axes).
    if nargin < 5, max_iter = 2000; end
    if nargin < 6, tol = 0.1; end % 0.1 mm tolerance

    q = q0(:);
    q_min = q_min(:);
    q_max = q_max(:);
    success = false;
    
    % Learning rate
    alpha = 0.5;
    % Damping factor (lambda) for Damped Least Squares
    lambda = 10.0; 

    for iter = 1:max_iter
        % 1. Forward Kinematics to get current position
        [T07, ~] = forwardKinematics(q);
        p_cur = T07(1:3, 4);

        % 2. Calculate position error
        e = target_pos(:) - p_cur;

        % 3. Checking convergence
        if norm(e) < tol
            success = true;
            fprintf('IK Converged in %d iterations. Error: %.4f mm\n', iter, norm(e));
            return;
        end

        % 4. Compute Geometric Jacobian
        J = jacobian(q, p_cur);

        % 5. Extract the position part of the jacobian (3x7)
        J_pos = J(1:3, :);
        
        % 6. Damped Least Squares (DLS) pseudoinverse
        % J_dls = J^T * (J * J^T + lambda^2 * I)^-1
        % This handles 7-DOF redundancy and avoids singularities
        J_dls = J_pos' / (J_pos * J_pos' + (lambda^2) * eye(3));
        dq = J_dls * e;

        % 7. Update and clamp to joint limits (CORRECTED)
        q = q + alpha * dq;
        q = max(q_min, min(q, q_max)); 
    end

    warning('IK did not converge in %d iterations (error = %.4f mm)', max_iter, norm(e));
end