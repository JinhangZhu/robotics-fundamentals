function q = FindqSet(q1,q2,q3,q4,q5,lastq)
%   Find the best q set for the specific position
%   The q variables are deduced from inverse kinematics
    if isnan(lastq) %   No last q: start position
        if size(q2,2) == 1
            q = [q1,q2,q3,q4,q5]';
        else
            temp = [q1,q2(1),q3(1),q4(1),q5;q1,q2(2),q3(2),q4(2),q5]';
            % column 1 closer to previous
            if rand > 0.5
                q = temp(:,1);
            else
                q = temp(:,2);
            end
        end
    else    %   Last q exists: sampled points or end point
        if size(q2,2) == 1
            q = [q1,q2,q3,q4,q5]';
        else
            temp = [q1,q2(1),q3(1),q4(1),q5;q1,q2(2),q3(2),q4(2),q5]';
            % column 1 closer to previous
            if norm(lastq-temp(:,1)) <= norm(lastq-temp(:,2)) 
                q = temp(:,1);
            else
                q = temp(:,2);
            end
        end
    end
end