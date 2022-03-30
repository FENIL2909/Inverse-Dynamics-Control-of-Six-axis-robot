function J = jacob0(S,q)
% JACOB0 Calculates the space jacobian matrix of the robot corresponding to  
% the given Screw axis and joint variables of all the joints.
%
% Inputs: S - 6xn matrix consisting the screw axis of all n joints of the
%             robot
%         q - 1xn matrix consisting the joint variables of all n joint of
%             the robot
%
% Output: J - 6Xn space jacobian matrix 
%
% see also ADJOINT, TWIST2HT
%
% Author: Fenil Desai <fdesai@wpi.edu>
% Last modified: 11/03/2021
SIZE = size(S);
    TRANSFORMATIONS = containers.Map(1,eye(4));
    
    for i = 2:SIZE(2)
        TRANSFORMATIONS(i) = TRANSFORMATIONS(i-1)*twist2ht(S(:,i-1),q(i-1));
    end
    
    
   for j = 1:SIZE(2)
       J(:,j) = adjoint(S(:,j),TRANSFORMATIONS(j));
   end
end