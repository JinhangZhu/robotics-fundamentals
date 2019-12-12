function Oi = decideOi(pt_current,PT_GOAL,endpoints)
%DECIDEOI Determine which endpoint to go towards
%   Detailed explanation goes here
heuristic_distance_1 = norm(pt_current-endpoints(:,1))+...
    norm(endpoints(:,1)-PT_GOAL);
heuristic_distance_2 = norm(pt_current-endpoints(:,2))+...
    norm(endpoints(:,2)-PT_GOAL);

if heuristic_distance_1 < heuristic_distance_2
    Oi = endpoints(:,1);
else
    Oi = endpoints(:,2);
end

end

