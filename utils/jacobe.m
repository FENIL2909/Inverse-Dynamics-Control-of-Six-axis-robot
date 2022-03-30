function J_b = jacobe(S,M,q)    
% JACOBE Calculates the Body (in - end effector frame) jacobian matrix of 
% the robot corresponding to the given Screw axis and joint variables of 
% all the joints.
%
% Inputs: S - 6xn matrix consisting the screw axis of all n joints of the
%             robot in space frame
%         q - 1xn matrix consisting the joint variables of all n joint of
%             the robot
%         M - 4X4 Home Configuration Matrix
%
% Output: J_v - 6Xn Body (in end effector frame) jacobian matrix 
%
% Author: Fenil Desai <fdesai@wpi.edu>
% Last modified: 11/03/2021
    T = fkine(S,M,q,'space');
    R = [T(1,1),T(1,2),T(1,3); T(2,1), T(2,2), T(2,3); T(3,1), T(3,2), T(3,3)];
    p = [T(1,4), T(2,4), T(3,4)]';
    
    p_bracket = [0,-p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    K = p_bracket*R;
    O = [0 0 0; 0 0 0; 0 0 0];
    AD_T = [R,O;K,R];

    SIZE = size(S);
    TRANSFORMATIONS = containers.Map(1,eye(4));
    
    for i = 2:SIZE(2)
        TRANSFORMATIONS(i) = TRANSFORMATIONS(i-1)*twist2ht(S(:,i-1),q(i-1));
    end
    
   for j = 1:SIZE(2)
       J(:,j) = adjoint(S(:,j),TRANSFORMATIONS(j));
   end
   
   J_b = pinv(AD_T)*J;
end
