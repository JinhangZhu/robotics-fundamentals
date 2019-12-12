function mode = checkoff(pt_current,curve,PT_GOAL,SENSOR_RANGE,obstacles,LEN_STEP,endpoints)
%checkoff Check the condition to terminate boundary following
%   Happens in boundary-following mode and turns the mode into
%   motion-to-goal mode if true.
min_dist = readshortest(pt_current,curve);

d_reach = norm(pt_current-PT_GOAL)-SENSOR_RANGE;
d_followed = norm(curve(:,1)-PT_GOAL);
n_curve = size(curve,2);
for i = 1:n_curve
    if norm(curve(:,i)-PT_GOAL) < d_followed
        d_followed = norm(curve(:,i)-PT_GOAL);
    else
        continue
    end
end

if ~isnan(endpoints)
    if checkIntersect(pt_current,PT_GOAL,endpoints)
        mode = 1;
    else
        mode = 0;
        return;
    end
end

if d_reach < d_followed && min_dist >= LEN_STEP*20
    mode = 0;
else
    mode = 1;
end

end

