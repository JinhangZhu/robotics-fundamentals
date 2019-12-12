function [frames_pos,HT] = ComputeFramePosition(DH_param_whole)
% Compute frames positions at under certain DH parameters
% draw one robot at a call
n_frames = size(DH_param_whole,1);  % get row count, number of frames
% frames position matrix; 3*n_frames
% e.g. frames_pos(:,1)=[x,y,z]' of frame 1
frames_pos = zeros(3,n_frames);
frames_pos0 = [0,0,0]';

for i = 1:n_frames
    T_till_now = modified_DH_whole(DH_param_whole(1:i,:)); % single transformation matrix
    frames_pos(:,i) = T_till_now(1:3,4);
end

HT = T_till_now;
frames_pos = [frames_pos0 frames_pos];

end