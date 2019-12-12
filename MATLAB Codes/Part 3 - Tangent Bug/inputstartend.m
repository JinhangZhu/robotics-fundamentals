function [pt_start,pt_goal] = inputstartend()
%INPUTSTARTEND Call this function to get the two endpoints
%   Tell the user to input start x, start y, end x and end y separately.
%   And the function combines them to make two point vectors.
pt_start = zeros(2,1);
pt_goal = zeros(2,1);

%
disp('Input coordinates of start point and goal point...')

while true
    start_x = input('Input x of start point: ');
    if isempty(start_x)
        continue
    else
        break
    end
end
while true
    start_y = input('Input y of start point: ');
    if isempty(start_y)
        continue
    else
        break
    end
end
while true
    goal_x = input('Input x of start point: ');
    if isempty(goal_x)
        continue
    else
        break
    end
end
while true
    goal_y = input('Input y of start point: ');
    if isempty(goal_y)
        continue
    else
        break
    end
end

pt_start = [start_x;start_y];
pt_goal = [goal_x;goal_y];
clc

end