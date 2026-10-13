%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end
    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            % Get current Tool pose from geometric model
            bTt = self.gm.getToolTransformWrtBase();
            
            % --- 1. Linear Error Calculation ---
            % e_linear = P_goal - P_tool
            lin_err = bTg(1:3,4) - bTt(1:3,4);
            
            % --- 2. Rotational Error Calculation (Using Custom RotToAngleAxis) ---
            % a. Compute relative rotation matrix between goal and tool frames
            % tRg = bRt' * bRg
            rot_err = bTt(1:3,1:3)' * bTg(1:3,1:3);  
            
            % b. Convert to Axis-Angle using the PROVIDED CUSTOM FUNCTION
            % Returns h (axis) and theta (angle)
            [h, theta] = RotToAngleAxis(rot_err);
            
            % c. Compute rotational error vector rho = h * theta
            % Force h to be a column vector (3x1) to handle the theta=0 case in their code
            h = h(:); 
            rho = h * theta;
            
            % d. Transform rho back to base frame: b_e_rot = bRt * rho
            Rotational_error = bTt(1:3,1:3) * rho;
            
            
            % --- 3. Compute Desired Velocities ---
            % x_dot : cartesian reference for inverse kinematic control
            v_des = self.k_l * lin_err;
            omega_des = self.k_a * Rotational_error; 
            
            % Assemble x_dot (Angular velocity first)
            x_dot = [omega_des; v_des];
            
        end
    end
end