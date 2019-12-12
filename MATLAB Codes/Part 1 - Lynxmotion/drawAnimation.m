function [frame_plot,ee_points,ee_info] = drawAnimation(frames_pos,HT,title_animation,streeFlag)
% draw one robot at a call

n_frames = size(frames_pos,2) - 1;

% Convert negative zeros to zeros
HT = del_nega_zeros(HT);
pt_ee = [frames_pos(1,n_frames+1),frames_pos(2,n_frames+1),frames_pos(3,n_frames+1)];
angle_elevation = round(atand(HT(3,1)/HT(3,3))+90);

% Plot
frame_plot = plot3(frames_pos(1,:),frames_pos(2,:),frames_pos(3,:),'bo-','Linewidth',2);
hold on
ee_points = plot3(pt_ee(1),pt_ee(2),pt_ee(3),'r.');
hold on

if streeFlag
    str_ee_position = sprintf('(%0.4f, %0.4f, %0.4f, %d degrees)',pt_ee(1),pt_ee(2),pt_ee(3),angle_elevation);
    ee_info = text(pt_ee(1) + 0.002,pt_ee(2) + 0.002,pt_ee(3) + 0.002,str_ee_position);
end

title(title_animation);

axis equal
hold on
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
grid on
% pause(time_pause);
% delete(frame_plot);
