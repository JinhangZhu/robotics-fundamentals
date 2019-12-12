%%%%%%%%%%%%%%%%%%%%
%   Author: Jinhang Zhu
%   Description:
%       Input:  psi(theta2+3+4), px, py, pz
%       Output: theta1, theta2, theta3, theta4
%       Verification: Transform matrix

%%  Load specifications
load('specifications.mat');

%%  Inverse kinematics
while true
    disp('========================================')
    disp('Input position');
    px = input('Px: ');
    py = input('Py: ');
    pz = input('Pz: ');
    disp('========================================')
    disp('Input orientation');
    disp('orientation: angle of elevation of');
    disp('end-effector or q2 + q3 + q4')
    psi = input('Psi: ');
    
    %   Call ik_func module to calculate joint values q1,q2,q3,q4
    [P,flag,q1,q2,q3,q4] = ik_func([px,py,pz],psi,a,d);
    if isnan(flag)
        disp('========================================')
        disp('Point NOT in workspace! Re-input now!')
        continue
    end
    
    %   Solution discussion
    if size(q2,2) == 1
        isWorkspace = q1 >= q1_range(1) && q1 <= q1_range(2) &&...
            q2 >= q2_range(1) && q2 <= q2_range(2) &&...
            q3 >= q3_range(1) && q3 <= q3_range(2) &&...
            q4 >= q4_range(1) && q4 <= q4_range(2);
        if ~isWorkspace
            disp('========================================')
            disp('Point NOT in workspace! Re-input now!')
            continue
        else
            q = [q1,q2,q3,q4,q5]';
            break
        end
    else % 2 theta 2 values
        % check 1st set
        isWorkspace_1 = q1 >= q1_range(1) && q1 <= q1_range(2) &&...
            q2(1) >= q2_range(1) && q2(2) <= q2_range(2) &&...
            q3(1) >= q3_range(1) && q3(2) <= q3_range(2) &&...
            q4(1) >= q4_range(1) && q4(2) <= q4_range(2);
        if isWorkspace_1
            % 1st solution set effective
            % check 2nd set
            isWorkspace_2 = q1 >= q1_range(1) && q1 <= q1_range(2) &&...
                q2(2) >= q2_range(1) && q2(2) <= q2_range(2) &&...
                q3(2) >= q3_range(1) && q3(2) <= q3_range(2) &&...
                q4(2) >= q4_range(1) && q4(2) <= q4_range(2);
            if isWorkspace_2
                % both solution sets effective
                q = [q1,q2(1),q3(1),q4(1),q5;q1,q2(2),q3(2),q4(2),q5]';
                break
            else
                % 1st set effective, but 2nd set not
                q = [q1,q2(1),q3(1),q4(1),q5]';
                break
            end
        else
            % 1st solution set not effective
            % check 2nd set
            isWorkspace_2 = q1 >= q1_range(1) && q1 <= q1_range(2) &&...
                q2(2) >= q2_range(1) && q2(2) <= q2_range(2) &&...
                q3(2) >= q3_range(1) && q3(2) <= q3_range(2) &&...
                q4(2) >= q4_range(1) && q4(2) <= q4_range(2);
            if isWorkspace_2
                % 2nd set effective, but 1st set not
                q = [q1,q2(2),q3(2),q4(2),q5]';
                break
            else
                % either of two sets effective
                disp('========================================')
                disp('Point NOT in workspace! Re-input now!')
                continue
            end
        end
    end
end

disp('========================================')
disp('Joint values')
q = round(q);
disp(q');

%%  Forward kinematics: Verification
if size(q,2)==1 % 1 set
    DH_param_whole = [alpha,a,q,d];
    figure(1);
    [frames_pos,HT] = ComputeFramePosition(DH_param_whole);
    drawAnimation(frames_pos,HT,'Result',1);
    xlim(xlim_range);
    ylim(ylim_range);
    zlim(zlim_range);
    T05 = del_nega_zeros(modified_DH_whole(DH_param_whole));
    disp('========================================')
    disp('Final matrix');
    disp(T05);
    if round(P,2) == round(T05,2)
        disp('Correct!')
    end
else % 2 sets
    DH_param_whole_1 = [alpha,a,q(:,1),d];
    DH_param_whole_2 = [alpha,a,q(:,2),d];
    figure(1);
    [frames_pos,HT] = ComputeFramePosition(DH_param_whole_1);
    drawAnimation(frames_pos,HT,'Result',1);
    [frames_pos,HT] = ComputeFramePosition(DH_param_whole_2);
    drawAnimation(frames_pos,HT,'Result',1);
    xlim(xlim_range);
    ylim(ylim_range);
    zlim(zlim_range);
    T05_1 = del_nega_zeros(modified_DH_whole(DH_param_whole_1));
    T05_2 = del_nega_zeros(modified_DH_whole(DH_param_whole_2));
    disp('========================================')
    disp('Final matrix');
    disp(T05_1);
    disp('or')
    disp(T05_2);
    if all(all(round(P,2) == round(T05_1,2))) && all(all(round(P,2) == round(T05_2,2)))
        disp('Both Correct!')
    end
end