function pt_next = straight2next(pt_current,pt_dest,LEN_STEP)
%STRAIGHT2NEXT Find the next point to move to in a straight line.
%   Find the next point based on the step size and angle of gradients. If
%   the distance between the current spot and destination is less than step
%   size, let pt_next be pt_dest.
pt_next = zeros(2,1);

%
angle = atan2(pt_dest(2)-pt_current(2),pt_dest(1)-pt_current(1));

if norm(pt_dest-pt_current) > LEN_STEP
    pt_next(1) = pt_current(1)+cos(angle)*LEN_STEP;
    pt_next(2) = pt_current(2)+sin(angle)*LEN_STEP;
else
    pt_next = pt_dest;
end

end