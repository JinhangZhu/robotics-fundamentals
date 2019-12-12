function min_dist = readshortest(pt_current,obstacles)
%readshortest Get the shortest distance between current point and the curve

n_curve = size(obstacles,2);
min_dist = Inf;
for i_obs = 1:n_curve
    temp_dist = norm(pt_current-obstacles(:,i_obs));
    if temp_dist < min_dist
        min_dist = temp_dist;
    end
end

end

