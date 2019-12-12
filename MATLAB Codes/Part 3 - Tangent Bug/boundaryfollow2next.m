function [pt_next,angle_next] = boundaryfollow2next(angle_previous,pt_current,pt_dest,LEN_STEP,obstacles)
%boundaryfollow2next Find next point on normal direction while following boudary
%   input:      pt_currrent, pt_dest (the most recent destination), curve
%   output:     pt_next
pt_next = zeros(2,1);

%
% angle_most_current = atan2(pt_dest(2)-pt_current(2),pt_dest(1)-pt_current(1));
angle_most_current = angle_previous;

%   Find the closest point on the curve to the current point
n_curve = size(obstacles,2);
min_dist = norm(pt_current-obstacles(:,1));
pt_closest = obstacles(:,1);
for i = 2:n_curve
    if norm(pt_current-obstacles(:,i)) < min_dist
        min_dist = norm(pt_current-obstacles(:,i));
        pt_closest = obstacles(:,i);
    else
        continue
    end
end

angle_closest = atan2(pt_closest(2)-pt_current(2),pt_closest(1)-pt_current(1));
if angle_closest > angle_most_current
    angle_next = angle_closest - pi/2;
else
    angle_next = angle_closest + pi/2;
end

pt_next(1) = pt_current(1)+cos(angle_next)*LEN_STEP;
pt_next(2) = pt_current(2)+sin(angle_next)*LEN_STEP;

end

