function [q] = KinematicSimulation(q, q_dot, dt, qmin, qmax)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - dt sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

    % Euler integration step: q(k+1) = q(k) + q_dot * dt
    q = (dt .* q_dot) + q;

    % Enforce joint limits
    for i = 1:length(q)
        if q(i) > qmax(i)
            q(i) = qmax(i);
        elseif q(i) < qmin(i)
            q(i) = qmin(i);
        end 
    end
end