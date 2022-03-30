% RBE 501 - Robot Dynamics - Spring 2021
% Final Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 05/02/2021
clear, clc, close all
addpath('utils');

% Run the script to create the robot's dynamic model
dynmodel

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


%% Inverse Dynamics Control
nPts = 100;
targetQ = zeros(n,nPts);

% Set the current joint variables
currentQ = zeros(1,n);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S,M,currentQ);

y = linspace(-0.4, 0.4, nPts);
x = 0.5 * ones(1,nPts);
omicron = linspace(0, pi, nPts);
z = 0.2 * sin(omicron);
path = [x; y; z];

% *** YOUR CODE HERE ***
fprintf('Calculating the Inverse Kinematics... ');
robot.plot(currentQ);
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Iterate over the target points
for ii = 1 : nPts
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S,M,currentQ,'space');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);
        
        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
        lambda = 0.05;
        deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);
                    
        currentQ = currentQ + deltaQ';       
        T = fkine(S,M,currentQ,'space');
        currentPose = T(1:3,4);
    end
    
    targetQ(:,ii) = currentQ;
end

fprintf('Done.\n');


% Now, for each pair of consecutive set points, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generate the Joint Torque Profiles... ');

% Initialize the arrays where we will accumulate the output of the robot
% dynamics, so that we can display it later
qtt = []; % Joint Variables
tau = [];

for jj = 1 : nPts - 1
    t0 = 0; tf = 0.5; % Starting and ending time of each trajectory
    N = 50;          % Number of intermediate setpoints
    t = linspace(t0, tf, N); % time vector
    
    q = zeros(n,N);   % joint variables
    qd = zeros(n,N);  % joint velocities
    qdd = zeros(n,N); % joint accelerations
    
    for ii = 1 : n
        % Calculate the coefficients of the quintic polynomial
        a = quinticpoly(t0, tf, ...
            targetQ(ii,jj), targetQ(ii,jj+1), ...
            0, 0, 0, 0);
        
        % Generate the joint profiles (position, velocity, and
        % acceleration)
        q(ii,:) = a(1) + a(2) * t + a(3) * t.^2 + a(4) * t.^3 + a(5) * t.^4 + a(6) * t.^5;
        qd(ii,:) = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        qdd(ii,:) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end
    
    % Use the equations of motion to calculate the necessary torques to trace
    % the trajectory
    Ftipmat = zeros(N,6); % no end effector force
    taumat = InverseDynamicsTrajectory(q', qd', qdd', ...
        g, Ftipmat, Mlist, Glist, S);
    
    % Use the Forward Dynamics to simulate the robot behavior
    dt = tf/N;  % time step
    intRes = 1; % Euler integration constant
    [qt, qdt] = ForwardDynamicsTrajectory(q(:,1), qd(:,1), taumat, g, ...
        Ftipmat, Mlist, Glist, S, dt, ...
        intRes);
    
    qtt = [qtt; qt]; % Accumulate the results
    tau = [tau; taumat];
end

fprintf('Done.\n');

fprintf('Simulate the robot...');
title('Inverse Dynamics Control');
robot.plot(qtt(1:10:end,:));
fprintf('Done.\n');

% Display the Joint Torques
figure, hold on, grid on
plot((1:length(tau))*dt, tau(:,1), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,2), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,3), 'Linewidth', 2);
plot((1:length(tau))*dt, tau(:,4), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,5), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,6), 'Linewidth', 2);
xlim([0 max((1:length(tau))*dt)]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3','Joint 4', 'Joint 5', 'Joint 6'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');