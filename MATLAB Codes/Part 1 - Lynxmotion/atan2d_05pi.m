function result = atan2d_05pi(up,down)
%Return value of atand in the range of [-90,90]
if up || down
    result = atand(up/down);
else
    result = atan2d(up,down);
end

