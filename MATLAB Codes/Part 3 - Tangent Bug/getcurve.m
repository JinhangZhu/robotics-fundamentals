function [isExist,curve,endpoints] = getcurve(obstacles,pt_current,PT_GOAL,SENSOR_RANGE)
%getcurve Get sensing curve and indicate endpoints
%   There may be multiple curves and we only care about the curve that
%   intersects with current-goal line (initially as m-line).

n_iter = 720;
angle_axis = linspace(0,2*pi,n_iter);
angle_step = angle_axis(2)-angle_axis(1);

circ_scanned = Inf(2,n_iter);       % Circumference points coordinates
circ_occupied = zeros(1,n_iter);    % Check the occupancy of circ_scanned

n_obs = size(obstacles,2);

for i_angle = 1:n_iter
    len_closest = SENSOR_RANGE;
    for i_obs = 1:n_obs
        pt_obs = obstacles(:,i_obs);
        angle_obs = atan2(pt_obs(2)-pt_current(2),...
            pt_obs(1)-pt_current(1));
        
        % Find obstacle points at the same angle
        if abs(angle_obs - angle_axis(i_angle)) < angle_step
            % Assumed as the same if angles are very close
            len_temp = norm(pt_current-pt_obs);
            if len_temp < len_closest
                circ_scanned(:,i_angle) = pt_obs;
                len_closest = len_temp;
                if ~circ_occupied(i_angle)  % First-time scanned
                    circ_occupied(i_angle) = true;
                end
            end
        end
    end
end

% Split the circumference points into curves
% and chooose the intersecting curve.
diff_occupied = [diff(circ_occupied) ...
    circ_occupied(1)-circ_occupied(n_iter)];

% Means filter to eliminate odd ajoint -1 and 1, core: 3
temp = zeros(1,n_iter);
for i_filter = 1:n_iter
    if i_filter == 1
        temp(i_filter) = diff_occupied(1)+...
        diff_occupied(n_iter)+diff_occupied(2);
    elseif i_filter == n_iter
        temp(i_filter) = diff_occupied(n_iter)+...
        diff_occupied(n_iter-1)+ diff_occupied(1);
    else
        temp(i_filter) = diff_occupied(i_filter)+...
        diff_occupied(i_filter-1)+diff_occupied(i_filter+1);
    end
end
diff_occupied = temp;

id_end_pos = find(diff_occupied == 1) + 1;
id_end_neg = find(diff_occupied == -1);
if ~circ_occupied(n_iter)
    id_end_pos(id_end_pos > n_iter) = 1;
end
id_end_pos = sort(id_end_pos);
id_end_neg = sort(id_end_neg);

if isempty(id_end_pos) || isempty(id_end_neg)
    curve = NaN;
    endpoints = NaN;
    isExist = false;
    return
end


% Use min and max
id_pos = min(id_end_pos);
id_neg = max(id_end_neg);

if id_pos == id_neg     % only one scanned point  
    curve = NaN;
    endpoints = NaN;
    isExist = false;
    return
end
curve = circ_scanned(:,id_pos:id_neg);
curve = curve(:,~isinf(curve(1,:)));
%     endpoints = [circ_scanned(:,id_pos) circ_scanned(:,id_neg)];
endpoints = [curve(:,1) curve(:,size(curve,2))];
if checkIntersect(pt_current,PT_GOAL,endpoints)
%         curve = circ_scanned(:,(min(id_pos,id_neg):max(id_pos,id_neg)));
    isExist = true;
else
    curve = NaN;
    endpoints = NaN;
    isExist = false;
end
% 
% % Curve: from id_end_pos to id_end_neg
% for i = 1:size(id_end_pos,2)
%     id_pos = id_end_pos(i);
%     id_temp = id_pos;
%     while true
%         if any(id_end_neg(:) == id_temp)
%             break;
%         else
%             id_temp = id_temp + 1;
%             if id_temp > n_iter
%                 id_temp = 1;
%             end
%         end
%     end
%     id_neg = id_temp;
%     if id_pos == id_neg     % only one scanned point  
%         continue
%     end
%     curve = circ_scanned(:,(min(id_pos,id_neg):max(id_pos,id_neg)));
%     curve = curve(:,~isinf(curve(1,:)));
% %     endpoints = [circ_scanned(:,id_pos) circ_scanned(:,id_neg)];
%     endpoints = [curve(:,1) curve(:,size(curve,2))];
%     if checkIntersect(pt_current,PT_GOAL,endpoints)
% %         curve = circ_scanned(:,(min(id_pos,id_neg):max(id_pos,id_neg)));
%         isExist = true;
%         break;  % Find intersecting curve, end loop.
%     else
%         curve = NaN;
%         endpoints = NaN;
%         isExist = false;
%     end
% end

end
