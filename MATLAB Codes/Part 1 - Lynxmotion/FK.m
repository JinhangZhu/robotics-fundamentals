%%%%%%%%%%%%% Jinhang Zhu %%%%%%%%%%%
%%%%%%%%%%% Robotics Fundamentals %%%%%%%%%
%%%%%%%%%%%%%% October 2019 %%%%%%%%%%%%%
% The following code simulates the forward kinematics of a 4DOF Lynxmotion 
% robot. Figure 2 shows its movement from start to end
% position, Figure 1 shows the location of its end effector at points of
% its trajectory and Figure 2,3 shows the maximum potential workspace of the
% arm's end effector.

close all
clc
disp('The following code simulates the forward kinematics of a 4DOF Lynxmotion')
disp('robot. Figure 2 shows its movement from start to end position,')
disp('Figure 1 shows the location of its end effector at points of its trajectory')
disp('and Figure 2,3 shows the maximum potential workspace of its end effector')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Specifications
% A series of joint angles
% The following variables are defined in the form of column-vectors with
% 4 rows each. Each row represents a different position (angle) of the joint.
q1 = [60 70 80 90]';
q2 = [-30 -35 -40 -45]';
q3 = [45 55 60 65]';
q4 = [-60 -55 -50 -45]';

load('specifications.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Trajectory and the arm based on 4 statuses
% Prepare end-effector positions coordinates matrix
% columns:  x, y, z
% rows:     4 statuses
pt_ee = zeros(3,4);

% Relative position of end-effector in frame 4
p_rel_in_4 = [0 d5 0 1]';

% Prepare coordinates matrices of all frames
% columns:  frame 0-5
% rows:     4 statuses
frames_x = zeros(6,4);
frames_y = zeros(6,4);
frames_z = zeros(6,4);

for i = 1:4
    % Transformations between frames
    T01 = modified_DH_single([alpha(1),a(1),q1(i),d(1)]); % reveal frame 1 position
    T12 = modified_DH_single([alpha(2),a(2),q2(i),d(2)]); % reveal frame 2 position
    T23 = modified_DH_single([alpha(3),a(3),q3(i),d(3)]); % reveal frame 3 position
    T34 = modified_DH_single([alpha(4),a(4),q4(i),d(4)]); % reveal frame 4 position
    % Get forward kinemtics transformation matrices
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34; 
    P5_ee = T04*p_rel_in_4;
    % Fill ee position matrix
    pt_ee(:,i) = P5_ee(1:3,:);
    % Fill frames position matrices
    frames_x(:,i) = [0, T01(1,4), T02(1,4), T03(1,4), T04(1,4), pt_ee(1,i)]';   % x coordinate of frames
    frames_y(:,i) = [0, T01(2,4), T02(2,4), T03(2,4), T04(2,4), pt_ee(2,i)]';   % y coordinate of frames
    frames_z(:,i) = [0, T01(3,4), T02(3,4), T03(3,4), T04(3,4), pt_ee(3,i)]';   % z coordinate of frames
end

% Plot the trajectory
figure(1);
plot3(pt_ee(1,1),pt_ee(2,1),pt_ee(3,1),'rx')       % plot the first position of the robot's end effector
hold on
plot3(pt_ee(1,2:4),pt_ee(2,2:4),pt_ee(3,2:4),'x')       % plot the 3 following positions of the robot's end effector
title('Tip Trajectory'); xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

% Plot the robot arm
figure (2) 
for i = 1:4
    plot3(frames_x(:,i),frames_y(:,i),frames_z(:,i),'o-','Linewidth',2)
    axis equal
    hold on
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    % Mark start and end positions
    text(pt_ee(1,1),pt_ee(2,1),pt_ee(3,1),'x'); 
    text(pt_ee(1,1) + 0.002,pt_ee(2,1) + 0.002,pt_ee(3,1) + 0.002,'ptStart');
    text(pt_ee(1,4),pt_ee(2,4),pt_ee(3,4),'x'); 
    text(pt_ee(1,4) + 0.002,pt_ee(2,4) + 0.002,pt_ee(3,4) + 0.002,'ptEnd');
    pause(0.05)
%     hold off
    pause(0.1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Workspace
% Similarly, we extend the test sample to many points over the whole [0,360]
numStatus = 18;
q1_axis = linspace(q1_range(1),q1_range(2),numStatus);
q2_axis = linspace(q2_range(1),q2_range(2),numStatus);
q3_axis = linspace(q3_range(1),q3_range(2),numStatus);
q4_axis = linspace(q4_range(1),q4_range(2),numStatus);

% Prepare end-effector positions coordinates matrix
% columns:  x, y, z
% rows:     numStatus^4 statuses
pt_ee = zeros(3,numStatus^4);
% Count of end-effector position indices
count = 0;

tic
% Loop: Fill the coordinates matrix of the robot
for i_link1 = 1:numStatus	% for q1
    for i_link2 = 1:numStatus   % for q2
        for i_link3 = 1:numStatus   % for q3
            for i_link4 = 1:numStatus   % for q4
                count = count + 1;
                q = [q1_axis(i_link1),q2_axis(i_link2),q3_axis(i_link3),q4_axis(i_link4)]';
                T04 = modified_DH_whole([alpha(1:4,:),a(1:4,:),q,d(1:4,:)]);
                P5_ee = T04*p_rel_in_4;
                pt_ee(:,count) = P5_ee(1:3,:);
            end
        end
    end
end

% Plot the workspace
% 2D view
figure(3)
plot(pt_ee(1,:),pt_ee(2,:),'.');
xlabel('x (m)'); ylabel('y (m)');
axis equal
% 3D view
figure(4)
plot3(pt_ee(1,:),pt_ee(2,:),pt_ee(3,:),'.');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
axis equal
disp('Rendering time:');
toc