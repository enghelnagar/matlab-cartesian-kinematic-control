function [isRotationMatrix] = IsRotationMatrix(R)
% The function checks that the input R is a valid rotation matrix, that is 
% a valid element of SO(3).
% Return true if R is a valid rotation matrix, false otherwise. In the
% latter case, print a warning pointing out the failed check.

% Define the tolerance suggested in the assignment hint 
tol = 1e-3; 

% Default assumption
isRotationMatrix = true;

% Check 1: Must be 3x3
if ~isequal(size(R), [3, 3])
    warning('Matrix is not 3x3.');
    isRotationMatrix = false;
    return;
end

% Check 2: Orthogonality: R' * R = I 
I = eye(3);
ortho_check = R' * R;
% Check if all elements of the difference are within the tolerance
if ~all(all(abs(ortho_check - I) < tol))
    warning('Matrix is not orthogonal (R''*R != I).');
    isRotationMatrix = false;
    % We return after the first failure
end

% Check 3: Determinant: det(R) = +1 
det_R = det(R);
if abs(det_R - 1.0) > tol
    warning('Matrix determinant is not +1 (det(R) = %f).', det_R);
    isRotationMatrix = false;
end

end