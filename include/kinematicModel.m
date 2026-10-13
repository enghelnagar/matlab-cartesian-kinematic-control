%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
        RB_Jacobian % Rigid Body Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
                self.RB_Jacobian = zeros(6,6);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        
        function updateJacobian(self)
            %% Update Jacobian function
            % The function updates:
            % - J: end-effector jacobian matrix
            
            % 1. Get transformation matrices from the geometric model
            % Note: usually we need transformations up to joint i
            bTi = zeros(4,4,self.gm.jointNumber);
            for i = 1:self.gm.jointNumber
                bTi(:,:,i) = self.gm.getTransformWrtBase(i);
            end
            
            % 2. Extract FLANGE position (End-Effector, NOT Tool yet)
            P_end = bTi(1:3, 4, self.gm.jointNumber); 
            
            % 3. Compute Jacobian for the FLANGE
            for i = 1:self.gm.jointNumber
                % Extract joint axis (z-axis) and position
                K_i = bTi(1:3,3,i);     
                P_i = bTi(1:3,4,i);     
                
                if self.gm.jointType(i) == 0  % Rotational joint
                    J_v = cross(K_i, P_end - P_i);
                    J_w = K_i;
                elseif self.gm.jointType(i) == 1  % Prismatic joint
                    J_v = K_i;
                    J_w = [0; 0; 0];
                end
                
                % Update the Jacobian matrix columns
                self.J(:, i) = [J_w; J_v];
            end
            
            % 4. Compute Rigid Body Jacobian (Shift Matrix)
            % This shifts the Jacobian from Flange to Tool
            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);
            bRe = bTe(1:3,1:3);
            eTt = self.gm.eTt;
            eOt = eTt(1:3,4);
            
            % Calculate offset in Base frame
            bOt = bRe * eOt;
            bOt_x = bOt(1);
            bOt_y = bOt(2);
            bOt_z = bOt(3);
            
            % Skew symmetric matrix of the offset
            bOt_skew = [ 0      -bOt_z   bOt_y;
                         bOt_z   0      -bOt_x;
                        -bOt_y   bOt_x   0    ];
            I = eye(3);
            
            % Construct the Shift Matrix
            % [ I      0 ]
            % [ [r]x'  I ]  <- Note: [r]x' is equivalent to -[r]x which is correct for velocity shift
            self.RB_Jacobian = [ I            zeros(3,3);
                                 bOt_skew'    I         ];

            % 5. IMPORTANT: Apply the Shift to get the TOOL Jacobian
            % J_tool = Shift_Matrix * J_flange
            self.J = self.RB_Jacobian * self.J;
        end
    end
end