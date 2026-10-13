function [h,theta] = RotToAngleAxis(R)
% Given a rotation matrix this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% Check that R is a valid rotation matrix using IsRotationMatrix() 
if ~IsRotationMatrix(R)
    error('Input is not a valid rotation matrix.');
end
% Define a tolerance
tol = 1e-3; 

% Step 1: Calculate theta from the trace of R 
% tr(R) = 1 + 2*cos(theta)
trace_R = trace(R);

% Clip the argument for acos to [-1, 1] to avoid numerical errors
cos_theta = (trace_R - 1) / 2;
cos_theta = max(-1, min(1, cos_theta)); 
theta = acos(cos_theta); % theta will be in [0, pi] 

% Step 2: Find the axis h based on the value of theta
if abs(theta) < tol
    % Case 1: theta = 0. R = Identity matrix. 
    % Axis is "completely arbitrary". We return a zero vector.
    h = [0; 0; 1];
    theta = 0; 

elseif abs(theta - pi) < tol
    % Case 2: theta = pi. R is symmetric. 
    % We use the formula R = 2*h*h' - I 
        
    % Find i_dagger (i_max): select i with largest r_ii
    [~, i_max] = max([R(1,1), R(2,2), R(3,3)]);
    
    % Calculate magnitudes of all components
    h_mags = [sqrt((R(1,1) + 1) / 2);
              sqrt((R(2,2) + 1) / 2);
              sqrt((R(3,3) + 1) / 2)];
    
    h = zeros(3,1);
    
    % Set the reference component h(i_max)
    h(i_max) = h_mags(i_max);
    
    % Calculate other components using signs of off-diagonals
    % h_j = sgn(R(i_max, j)) * h_mags(j)
    
    if i_max == 1
        h(2) = sign(R(1,2)) * h_mags(2); % j=2
        h(3) = sign(R(1,3)) * h_mags(3); % j=3
    elseif i_max == 2
        h(1) = sign(R(2,1)) * h_mags(1); % j=1 
        h(3) = sign(R(2,3)) * h_mags(3); % j=3
    else % i_max == 3
        h(1) = sign(R(3,1)) * h_mags(1); % j=1 
        h(2) = sign(R(3,2)) * h_mags(2); % j=2 
    end
    
    
    theta = pi;
else
    % Case 3: General case (0 < theta < pi).
    % (sin(theta)) * h = vex((R - R') / 2) 
    
    % Calculate the axial vector of (R - R') / 2 
    axial_vector = 0.5 * [R(3,2) - R(2,3);
                          R(1,3) - R(3,1);
                          R(2,1) - R(1,2)];
    
    % Calculate h 
    h = axial_vector / sin(theta);   
end
end
 

function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1) 
% S_a = [  0  -a3   a2 ]
%       [  a3   0  -a1 ] 
%       [ -a2   a1   0 ]
a = [ S_a(3,2); 
      S_a(1,3); 
      S_a(2,1) ];
end