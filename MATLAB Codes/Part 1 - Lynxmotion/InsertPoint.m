function pt_insert = InsertPoint(pt_start,pt_end,radius,offset)
% Find a new point to pass through to avoid the obstacle

half_dist = norm(pt_end-pt_start)/2;
pt_centre = (pt_start+pt_end)/2;
dist2centre = sqrt((half_dist^2*radius^2)/(half_dist^2-radius^2))+offset;

% Normal vector from sphere centre
temp = (pt_centre - pt_start)/norm(pt_centre - pt_start);
normal_vector = null(temp');

pt_insert = pt_centre + dist2centre*normal_vector(:,1);

end