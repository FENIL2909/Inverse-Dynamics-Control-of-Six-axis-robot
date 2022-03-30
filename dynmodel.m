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

%% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

%% Create the manipulator
% Create the manipulator
n = 6; % degrees of freedom
L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.2; % Lenght of Link 2 [m]
L3 = 0.2; % Lenght of Link 3 [m]
L4 = 0.2; % Lenght of Link 4 [m]
L5 = 0.2; % Lenght of Link 5 [m]
L6 = 0.2; % Lenght of Link 6 [m]

r = 2.5e-2; % Link radius [m]

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

% Display the robot
qz = zeros(1,6);
robot.teach(qz, 'jaxes'), hold on;


%% Link transformations
M01 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 (L1/2) 1]';
M12 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 ((L1+L2)/2) 1]';
M23 = [0 0 -1 0; 0 1 0 0; 1 0 0 0; (L3/2) 0 (L2/2) 1]';
M34 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 ((L3+L4)/2) 1]';
M45 = [0 0 1 0; 0 -1 0 0; 1 0 0 0; (L5/2) 0 (L4/2) 1]';
M56 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 ((L5+L6)/2) 1]';
M67 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 (L6/2) 1]';

Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

%% Spatial Inertia Matrices

%mass in Kg
m1 = 5;
m2 = 1;
m3 = 0.5;
m4 = 0.5;
m5 = 0.5;
m6 = 0.5;

Ixx1 = (m1*(3*(r^2) + L1^2))/12;
Iyy1 = (m1*(3*(r^2) + L1^2))/12;
Izz1 = (m1*(r^2))/2;

Ixx2 = (m2*(3*(r^2) + L2^2))/12;
Iyy2 = (m2*(3*(r^2) + L2^2))/12;
Izz2 = (m2*(r^2))/2;

Ixx3 = (m3*(3*(r^2) + L3^2))/12;
Iyy3 = (m3*(3*(r^2) + L3^2))/12;
Izz3 = (m3*(r^2))/2;

Ixx4 = (m4*(3*(r^2) + L4^2))/12;
Iyy4 = (m4*(3*(r^2) + L4^2))/12;
Izz4 = (m4*(r^2))/2;

Ixx5 = (m5*(3*(r^2) + L5^2))/12;
Iyy5 = (m5*(3*(r^2) + L5^2))/12;
Izz5 = (m5*(r^2))/2;

Ixx6 = (m6*(3*(r^2) + L6^2))/12;
Iyy6 = (m6*(3*(r^2) + L6^2))/12;
Izz6 = (m6*(r^2))/2;

I1 = [Ixx1 0 0; 0 Iyy1 0; 0 0 Izz1];
I2 = [Ixx2 0 0; 0 Iyy2 0; 0 0 Izz2]; 
I3 = [Ixx3 0 0; 0 Iyy3 0; 0 0 Izz3]; 
I4 = [Ixx4 0 0; 0 Iyy4 0; 0 0 Izz4];
I5 = [Ixx5 0 0; 0 Iyy5 0; 0 0 Izz5]; 
I6 = [Ixx6 0 0; 0 Iyy6 0; 0 0 Izz6];

O3 = [0 0 0; 0 0 0; 0 0 0];

I = eye(3);

G1 = [I1,O3;O3,(m1*I)]; % ...; Spatial Inertia Matrix for link 1
G2 = [I2,O3;O3,(m2*I)];% ...; Spatial Inertia Matrix for link 2
G3 = [I3,O3;O3,(m3*I)];% ...; Spatial Inertia Matrix for link 3
G4 = [I4,O3;O3,(m4*I)]; % ...; Spatial Inertia Matrix for link 4
G5 = [I5,O3;O3,(m5*I)];% ...; Spatial Inertia Matrix for link 5
G6 = [I6,O3;O3,(m6*I)];% ...; Spatial Inertia Matrix for link 6
    
Glist = cat(3, G1, G2, G3, G4, G5, G6);