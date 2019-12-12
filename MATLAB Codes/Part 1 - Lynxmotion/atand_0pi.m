function result = atand_0pi(fraction)
%Return value of atand in the range of [0,180]
if atand(fraction) >= 0
    result = atand(fraction);
else
    result = atand(fraction) + 180;
end

