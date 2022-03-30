function J_a = jacoba(S,M,q)
% JACOBA Calculates the analytical jacobian matrix of the robot 
% corresponding to the given Screw axis in space frame and joint variables 
% of all the joints.
%
% Inputs: S - 6xn matrix consisting the screw axis of all n joints of the
%             robot expressed in space frame
%         q - 1xn matrix consisting the joint variables of all n joint of
%             the robot
%
% Output: J_a - 3Xn analytical jacobian matrix (velocity component) 
%
% Author: Fenil Desai <fdesai@wpi.edu>
% Last modified: 11/03/2021
    SIZE = size(S);
    TRANSFORMATIONS = containers.Map(1,eye(4));
    
    for i = 2:SIZE(2)
        TRANSFORMATIONS(i) = TRANSFORMATIONS(i-1)*twist2ht(S(:,i-1),q(i-1));
    end
    
    
   for j = 1:SIZE(2)
       J(:,j) = Adjoint(TRANSFORMATIONS(j))*S(:,j);
   end
   
   T = fkine(S,M,q,'space');
   p = [T(1,4), T(2,4), T(3,4)]';
   p_bracket = [0,-p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
   
   J_sv = [J(4,:);J(5,:);J(6,:)];
   J_sw = [J(1,:);J(2,:);J(3,:)];
   
   J_a = J_sv - (p_bracket*J_sw);
end