% RBE 501 - Robot Dynamics - Fall 2021
% Worcester Polytechnic Institute
% Final Exam
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 11/30/2021
clear, clc, close all
addpath('utils');

%% *** PUT THE SECOND DIGIT OF YOUR WPI ID HERE ***
secondDigitID = 4;

%% Create the manipulator
n = 6; % degrees of freedom
L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.2; % Lenght of Link 2 [m]
L3 = 0.2; % Lenght of Link 3 [m]
L4 = 0.2; % Lenght of Link 4 [m]
L5 = 0.2; % Lenght of Link 5 [m]
L6 = 0.2; % Lenght of Link 6 [m]

if secondDigitID <= 4
    robot = SerialLink([Revolute('d', L1,     'a', 0,  'alpha', pi/2), ...
        Revolute('d', 0,      'a', L2, 'alpha', 0,     'offset', pi/2), ...
        Revolute('d', 0,      'a', 0,  'alpha', pi/2), ...
        Revolute('d', L3+L4,  'a', 0,  'alpha', -pi/2), ...
        Revolute('d', 0,      'a', 0,  'alpha', -pi/2,  'offset', -pi/2), ...
        Revolute('d', -L5-L6, 'a', 0,  'alpha', pi)], ...
        'name', 'RBE 501 Final Exam Fall 2021');
else
    robot = SerialLink([Revolute('d', L1,     'a', 0,  'alpha', -pi/2), ...
        Revolute('d', 0,      'a', L2, 'alpha', 0,     'offset', -pi/2), ...
        Revolute('d', 0,      'a', 0,  'alpha', -pi/2), ...
        Revolute('d', L3+L4,  'a', 0,  'alpha', pi/2), ...
        Revolute('d', 0,      'a', 0,  'alpha', pi/2,  'offset', pi/2), ...
        Revolute('d', -L5-L6, 'a', 0,  'alpha', pi)], ...
        'name', 'RBE 501 Final Exam Fall 2021');
end

%% Inverse Kinematics
% Define the path that we wish the robot to trace
fprintf('Generating task space path... ');
nPts = 100;
y = linspace(-0.4, 0.4, nPts);
x = 0.5 * ones(1,nPts);
omicron = linspace(0, pi, nPts);
z = 0.2 * sin(omicron);
path = [x; y; z];
fprintf('Done.\n');

% Initialize the matrix to store the IK result
targetQ = zeros(n,nPts);

% Set the initial joint variables to zero
currentQ = zeros(1,n);

% Display the path
figure(1)
robot.plot(currentQ, 'jaxes'), hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('Inverse Kinematics')

%% *** YOUR CODE HERE ***

plotOn = false;
qList = [];
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

fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nPts) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Iterate over the target points
for ii = 1 : nPts
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nPts*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S, M, currentQ, 'space');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);
        lambda = 0.05;
        J_star = J_a'*pinv(J_a*J_a' + (lambda^2)*eye(3));
        deltaQ = J_star*(targetPose - currentPose);
        %deltaQ = pinv(J_a)*(targetPose - currentPose);

        currentQ = currentQ + deltaQ';
        
        T = fkine(S, M, currentQ, 'space');
        currentPose = T(1:3,4);
        qList(:,ii) = currentQ;

        if plotOn
            try
                figure(1)
                robot.teach(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end

%for animation!!!!
figure(3)
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
robot.plot(qList');

fprintf('\nTest passed successfully.\n');
