%
%   Author: Jinhang
%   Description: Implement a free motion along the viapoints
%                The trajectory is generated through polynomial curve
%                fitting.
%


%%  Load specifications & Global variables
close all
clc
load('specifications.mat');
psi = -90;          % Angle of elevation
t_interval = 0.01;  % Time interval
M = 60;             % Number of samples
motionFlag = 0;     % Flag to specify varying-speed motion
n_frames = size(alpha,1);   % Number of frames
n_viapoints = size(viapoints,2);    % Number of viapoints

%%  Polynomial curve fitting of viapoints
pts = zeros(3,(n_viapoints-1)*M+n_viapoints);
poly_coeff_ZofY = polyfit(viapoints(2,:),viapoints(3,:),5);
y_axis = linspace(viapoints(2,1),viapoints(2,n_viapoints),size(pts,2));
poly_model_ZofY = polyval(poly_coeff_ZofY,y_axis);
pts(1,:) = viapoints(1,1);
pts(2,:) = y_axis;
pts(3,:) = poly_model_ZofY;
n_pts = size(pts,2);

%%  Plot all viapoints
h = figure('units','normalized','outerposition',[0 0 1 1]);
plot3(viapoints(1,:),viapoints(2,:),viapoints(3,:),...
            'g*',...
            'MarkerSize',5,...
            'LineWidth',2); hold on
        
%%  Specify the very first position
pt_start = viapoints(:,1);
[P,flag,q1,q2,q3,q4] = ik_func(pt_start,psi,a,d);
q = FindqSet(q1,q2,q3,q4,q5,NaN);
        
%%  Compute inverse kinematics
frames_pos_pts = zeros(3,n_frames+1,n_pts);
HT_pts = zeros(4,4,n_pts);
q_pts = zeros(n_frames,n_pts);
for i = 1:n_pts
    [P,flag,q1,q2,q3,q4] = ik_func(pts(:,i),psi,a,d);
    q = FindqSet(q1,q2,q3,q4,q5,q);
    q_pts(:,i) = q;
    [frames_pos,HT] = ComputeFramePosition([alpha,a,q,d]);
    frames_pos(:,n_frames+1) = pts(:,i); % adjust end-effector position
    frames_pos_pts(:,:,i) = frames_pos;
    HT_pts(:,:,i) = HT;
end

%%  Animation: Trajectory
for i = 1:n_pts
    [frame_plot,ee_points,ee_info] = drawAnimation(frames_pos_pts(:,:,i),HT_pts(:,:,i),'Obstacle Avoidance',1);
    xlim(xlim_range);
    ylim(ylim_range);
    zlim(zlim_range);
    pause(t_interval);
    if i ~= n_pts
        delete(frame_plot);
        delete(ee_info);
    end
end