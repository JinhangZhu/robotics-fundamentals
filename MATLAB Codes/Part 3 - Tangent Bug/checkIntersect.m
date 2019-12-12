function isIntersect = checkIntersect(pt_current,PT_GOAL,endpoints)
%CHECKINTERSECT Check if the obstacle is blocking.
%   Check if current-goal line intersects
O1 = endpoints(:,1);
O2 = endpoints(:,2);
if pt_current(1) ~= PT_GOAL(1)
    multi = (O1(2)-lineEquation(pt_current,PT_GOAL,O1(1)))*...
            (O2(2)-lineEquation(pt_current,PT_GOAL,O2(1)));
else
    multi = (O1(1)-pt_current(1))*(O2(1)-PT_GOAL(1));
end

if multi > 0
    isIntersect = false;
else
    isIntersect = true;
end   

end

