function T = fkine(S,M,q,frame)
% FKINE Calculates the 4X4 homogenous transformation matrix corresponding 
% to the given Screw axis, joint variables of all the joints and frame.
%
% This function effectively calculates the forward kinematics of the robot.
%
% Inputs: S - 6xn matrix consisting the screw axis of all n joints of the
%             robot in space frame or body frame 
%         q - 1xn matrix consisting the joint variables of all n joint of
%             the robot
%         M - 4x4 matrix representing the home configuration of the robot
%             (SE(3))
%
% Output: T - 4X4 homogenous transformation matrix for the forward
%             kinematics (SE(3)) 
%
% see also TWIST2HT
%
% Author: Fenil Desai <fdesai@wpi.edu>
% Last modified: 11/03/2021
    SIZE  = size(S);
    T = eye(4);
    
    for i = 1:SIZE(2)
        a = twist2ht(S(:,i),q(i));
        T = T*a;
    end
    
    if strcmp(frame,'body')
        T = M*T;
    else
        T = T*M;
    end
end