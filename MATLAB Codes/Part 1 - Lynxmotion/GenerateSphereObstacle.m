function GenerateSphereObstacle(pt_start,pt_end,radius)
%   Generate a sphere-shape obstacle
pt_centre = (pt_start+pt_end)/2;

[x,y,z] = sphere;   % Generate a sphere
x = x*radius;       % Rescaled to specified radius
y = y*radius;
z = z*radius;

surf(x+pt_centre(1),y+pt_centre(2),z+pt_centre(3)); % Draw the sphere at centre
hold on

end