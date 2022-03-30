% RBE 501 - Robot Dynamics - Fall 2021
% Worcester Polytechnic Institute
% Final Exam
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 11/30/2021
clear, clc, close all
addpath('utils');

% Run the script to create the robot's dynamic model
dynmodel

%% Gravity Compensation 
% ** YOUR CODE HERE ***

% Screw axis of all the joints
S = [0 0 1 0 0 0;
    0 -1 0 L1 0 0;
    0 -1 0 (L1+L2) 0 0;
    1 0 0 0 (L1+L2) 0;
    0 -1 0 (L1+L2) 0 -(L3+L4);
    0 0 1 0 -(L3+L4) 0]';

%Home configuration of the robot
M = [1 0 0 0;
    0 -1 0 0;
    0 0 -1 0;
    (L3+L4) 0 ((L1+L2)-(L5+L6)) 1]';

q0 = zeros(6,1);  % initial configuration 
qd0 = zeros(6,1); % initial velocities

% Invoke the `GravityForces` function. This function calculates the inverse
% dynamics of the robot when the robot is not moving and the only force
% acting on it is gravity.
grav = GravityForces(q0, g, Mlist, Glist, S);

fprintf('Joint Torques: ');
fprintf('[%f %f %f %f %f %f] Nm\n', grav(1), grav(2), grav(3), grav(4), grav(5), grav(6));

% Simulate for 5 seconds
tf = 5;           % total simulation time
dt = 0.05;        % time step
taumat = repmat(grav', [tf/dt 1]);   % apply only the torque required for gravity comp
Ftipmat = zeros(size(taumat, 1), 6); % end effector wrench
intRes = 8;       % Euler integration step

% Invoke the `ForwardDynamicsTrajectory` to simulate the robot motion.
[qt,~] = ForwardDynamicsTrajectory(q0, qd0, taumat, g, ...
                                   Ftipmat, Mlist, Glist, S, dt, ...
                                   intRes);

title('Gravity Compensation');
robot.plot(qt);
