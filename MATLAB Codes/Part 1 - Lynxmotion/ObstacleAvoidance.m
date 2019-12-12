%
%   Author: Jinhang
%   Description: Avoid obstacles in line trajectory.
%

%%  Load specifications & Global variables
close all
clc
load('specifications.mat');
psi = -90;          % Angle of elevation
u = 0.1;            % Speed of end effector in eCartesian space
M = 60;             % Number of samples
motionFlag = 0;     % Flag to specify varying-speed motion
n_frames = size(alpha,1);   % Number of frames
n_viapoints = size(viapoints,2);    % Number of viapoints

%%  Interpolate viapoints
temp = zeros(3,2*n_viapoints-1);
for i_loop = 1:n_viapoints-1
    %%  Get inserted point position
    pt_start = viapoints(:,i_loop);
    pt_end = viapoints(:,i_loop+1);
    dist = norm(pt_end-pt_start);
    radius = dist/5;
    offset = radius;
    pt_insert = InsertPoint(pt_start,pt_end,radius,offset);
    temp(:,i_loop*2-1:(i_loop+1)*2-1) = [pt_start pt_insert pt_end];
end
viapoints = temp;
n_viapoints = size(viapoints,2);    % Number of viapoints

%%  Main loop: Get positions and plot trajectory
h = figure('units','normalized','outerposition',[0 0 1 1]);

%%  Specify the very first position
pt_start = viapoints(:,1);
plot3(pt_start(1),pt_start(2),pt_start(3),...
            'g*',...
            'MarkerSize',5,...
            'LineWidth',2); hold on
[P,flag,q1,q2,q3,q4] = ik_func(pt_start,psi,a,d);
q = FindqSet(q1,q2,q3,q4,q5,NaN);
for i_loop = 1:n_viapoints-1
    %%  Get 2 ajoint points from the set;
    %   Assign them to start position and end position separately
    pt_start = viapoints(:,i_loop);
    pt_end = viapoints(:,i_loop+1);
    dist = norm(pt_end-pt_start);
    radius = dist/5;
    offset = radius;
    
    %%  Generate sphere obstacle
    if mod(i_loop,2)
        GenerateSphereObstacle(viapoints(:,i_loop),viapoints(:,i_loop+2),radius);
        plot3(viapoints(1,i_loop+2),viapoints(2,i_loop+2),viapoints(3,i_loop+2),...
            'g*',...
            'MarkerSize',5,...
            'LineWidth',2); hold on
    end
      
    %%  Generate trajectory consisted of M sampled points between the 2 positions
    [pts,t_interval,vee] = GenerateLinearPath(pt_start,pt_end,u,M,motionFlag);
    
    frames_pos_samples = zeros(3,n_frames+1,M);
    HT_samples = zeros(4,4,M);
    q_samples = zeros(n_frames,M);
    
    %%  Compute inverse kinematics
    %   Start position
    [P,flag,q1,q2,q3,q4] = ik_func(pt_start,psi,a,d);
    q = FindqSet(q1,q2,q3,q4,q5,q);
    q_start = q;
    DH_param_whole_start = [alpha,a,q,d];
    [frames_pos_start,HT_start] = ComputeFramePosition(DH_param_whole_start);
    frames_pos_start(:,n_frames+1) = pt_start;  %   Adjust end-effector position
    % samples positions
    for i_sample = 1:M
        [P,flag,q1,q2,q3,q4] = ik_func(pts(:,i_sample),psi,a,d);
        q = FindqSet(q1,q2,q3,q4,q5,q);
        q_samples(:,i_sample) = q;
        [frames_pos,HT] = ComputeFramePosition([alpha,a,q,d]);
        frames_pos(:,n_frames+1) = pts(:,i_sample); % adjust end-effector position
        frames_pos_samples(:,:,i_sample) = frames_pos;
        HT_samples(:,:,i_sample) = HT;
    end
    % end position
    [P,flag,q1,q2,q3,q4] = ik_func(pt_end,psi,a,d);% Ensured to be effective
    q = FindqSet(q1,q2,q3,q4,q5,q);
    q_end = q;
    DH_param_whole_end = [alpha,a,q,d];
    [frames_pos_end,HT_end] = ComputeFramePosition(DH_param_whole_end);
    frames_pos_end(:,n_frames+1) = pt_end; % adjust end-effector position
    
    %%  Animation: Trajectory
    [frame_plot,ee_points,ee_info] = drawAnimation(frames_pos_start,HT_start,'Obstacle Avoidance',1);
    xlim(xlim_range);
    ylim(ylim_range);
    zlim(zlim_range);
    pause(t_interval);
    delete(frame_plot);
    delete(ee_info);
    
    for i_sample = 1:M
        [frame_plot,ee_points,ee_info] = drawAnimation(frames_pos_samples(:,:,i_sample),HT_samples(:,:,i_sample),'Obstacle Avoidance',1);
        xlim(xlim_range);
        ylim(ylim_range);
        zlim(zlim_range);
        pause(t_interval);
        delete(frame_plot);
        delete(ee_info);
    end
    
    [frame_plot,ee_points,ee_info] = drawAnimation(frames_pos_end,HT_end,'Obstacle Avoidance',1);
    xlim(xlim_range);
    ylim(ylim_range);
    zlim(zlim_range);
    if i_loop ~= n_viapoints-1
        delete(frame_plot);
        delete(ee_info);
    end
end