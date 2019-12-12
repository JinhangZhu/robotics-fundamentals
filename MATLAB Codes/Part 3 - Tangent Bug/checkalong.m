function mode = checkalong(pt_previous,pt_current,PT_GOAL,obstacles,LEN_STEP)
%checkalong Check the condition to start boundary following
%   Happens in motion-to-goal mode and turns the mode into
%   boundary-following mode if true.

min_dist = readshortest(pt_current,obstacles);

if norm(pt_current-PT_GOAL) > norm(pt_previous-PT_GOAL)...
        || min_dist < LEN_STEP*20
    mode = 1;
else
    mode = 0;
end

end

