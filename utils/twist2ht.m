function T = twist2ht(S,theta)
% TWIST2HT Calculates the 4x4 homogenous transformation matrix corresponding 
% to a Screw-axis of a joint [S] and the joint variable [theta].
%
% This function takes into account whether the joint in consideration is a
% revolute joint of prismatic joint.
%
% Inputs: S - a 6X1 vector representing the screw axis of the joint
%         theta - a scaler representing the joint variable
%
% Output: T - a 4X4 homogenous transformation matrix (SE(3))
%
% see also AXISANGLE2ROT
%
% Author: Fenil Desai <fdesai@wpi.edu>
% Last modified: 11/03/2021
format longg
    omega = [S(1) S(2) S(3)];
    v = [S(4) S(5) S(6)]';
    omega_bracket = [0,-omega(3),omega(2); omega(3),0,-omega(1); -omega(2), omega(1),0];
    omega_norm = norm(omega);
    vel_norm = norm(v);
    c = [0 0 0];
    d = [1];
    I = eye(3);
    
    
    if omega_norm >= 0.999999999999
        a = axisangle2rot(omega,theta);
        b = ((I*theta) + ((1-cos(theta))*omega_bracket) + ((theta - sin(theta))*(omega_bracket*omega_bracket)))*v;
        T = [a,b;c,d];
        
    elseif (omega_norm <= 0.0000000000001) && (vel_norm >= 0.999999999999) 
        a = I;
        b = v*theta;
        T = [a,b;c,d];
    end
end