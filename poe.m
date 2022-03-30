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

% Display the manipulator in the home configuration
qz = zeros(1,6);
robot.teach(qz, 'jaxes');
hold on;

%% *** YOUR CODE HERE ***

plotOn = true;
nTests = 20; % number of random test configurations

% Joint limits
qlim = [-pi  pi;  % q(1)
        -pi  pi;  % q(2)
        -pi  pi;  % q(3)
        -pi  pi;  % q(4)
        -pi  pi;  % q(5)
        -pi  pi]; % q(6)

%% Part A - Calculating the screw axes

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


%% Part B - Calculating the forward kinematics with the Product of Exponentials formula
fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Testing the forward kinematics for 20 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    %Generating a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    %Calculating the forward kinematics
    T = fkine(S,M,q,'space');
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    %Checking the correctnedd of the fkine function developed by me
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');