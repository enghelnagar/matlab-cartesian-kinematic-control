%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    % eTt (OPTIONAL) add a tool to the model rigid attached to the
    % end-effector
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end
    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        
        function updateDirectGeometry(self, q)
            % Update the matrices iTj based on the joint positions q
            self.q = q; % Store the joint positions
            
            % Iterate through each joint
            for i = 1:self.jointNumber
                % Get the base transformation for joint i
                R_i_0 = self.iTj_0(1:3, 1:3,i);  % Extract the rotation part
                P_i_0 = self.iTj_0(1:3, 4,i);    % Extract the position part
                
                if self.jointType(i) == 0  % Rotational joint
                    theta = q(i);
                    % Rotation matrix around Z-axis
                    R_delta = [cos(theta), -sin(theta), 0;
                               sin(theta),  cos(theta), 0;
                               0,           0,          1];
                    
                    R_i = R_i_0 * R_delta; % Update the rotation
                    P_i = P_i_0;           % Position remains the same (for rotational)
                    
                elseif self.jointType(i) == 1  % Prismatic joint
                    % Compute translation along the Z-axis of the frame
                    K_i = R_i_0(:,3);      % Translation direction (Z-axis)
                    R_i = R_i_0;           % Rotation remains the same
                    P_i = P_i_0 + K_i * q(i);  % Update the position
                end
                
                % Update the transformation matrix for joint i
                self.iTj(:,:,i) = [R_i P_i; 0 0 0 1];
            end
        end
        
        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint
            
            % Initialize the base transformation to the identity matrix
            bTk = eye(4);
            
            % Iterate through the transformation matrices from link 1 to link k
            for i = 1:k
                % Multiply the transformation matrix from the previous frame to the current frame
                bTk = bTk * self.iTj(:,:,i);
            end
        end
                
        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % Inputs : None 
            % bTt : transformation matrix from the manipulator base to the tool
            
            % 1. Get transformation from Base to End-Effector (Last Joint)
            % Reuse the function we just wrote to avoid code duplication
            bTe = self.getTransformWrtBase(self.jointNumber);
            
            % 2. Multiply by End-Effector to Tool transformation (eTt)
            bTt = bTe * self.eTt;
        end
    end
end