%%   TANGENTBUG.M
%   Author: Jinhang Zhu
%   Date:   8 Dec 2019
%   Description:    Implement TangentBug path planning algorithm.
%

%%  Global variables
global SENSOR_RANGE     % Sensing range of the range sensor
global LEN_STEP         % Step length of a movement of the robot
global PT_START         % Start point coordinates
global PT_GOAL          % Goal point coordinates

SENSOR_RANGE = 0.5;
LEN_STEP = 0.01;
[PT_START,PT_GOAL] = inputstartend();
left_bottom = [min(PT_START(1),PT_GOAL(1));min(PT_START(2),PT_GOAL(2))];
right_top = [max(PT_START(1),PT_GOAL(1));max(PT_START(2),PT_GOAL(2))];

disp(['Start from: (',num2str(PT_START(1)),',',num2str(PT_START(2)),')']);
disp(['Go to: (',num2str(PT_GOAL(1)),',',num2str(PT_GOAL(2)),')']);
close
h = figure('units','normalized','outerposition',[0 0 1 1]);
plot(PT_START(1),PT_START(2),'r+',"LineWidth",2,"MarkerSize",5); hold on
plot(PT_GOAL(1),PT_GOAL(2),'r*',"LineWidth",2,"MarkerSize",5); hold on
title(['Step length: ',num2str(LEN_STEP),...
    ';  Sensor range: ',num2str(SENSOR_RANGE)]);

%%  Create obstacles in the plane
[obstacles,obs_1,obs_2,obs_3] = createobstacles(PT_START,PT_GOAL);
plot(PT_START(1),PT_START(2),'r+',"LineWidth",2,"MarkerSize",5); hold on
plot(PT_GOAL(1),PT_GOAL(2),'r*',"LineWidth",2,"MarkerSize",5); hold on
plot(obs_1(1,:),obs_1(2,:),'k'); hold on
plot(obs_2(1,:),obs_2(2,:),'k'); hold on
plot(obs_3(1,:),obs_3(2,:),'k'); hold on

tic
%%  Main loop
%   motion-to-goal & boundary-following
x_path = [];                        % Record of path's x
y_path = [];                        % Record of path's y
x_path = [x_path PT_START(1)];      % Start point stored
y_path = [y_path PT_START(2)];      % 

pt_current = PT_START;
pt_previous = pt_current;

mode = 0;   % mode = 0, do motion-to-goal
            % mode = 1, do boundary-following

while true
    robot_plot = plot(pt_current(1),pt_current(2),...   % Current location
        'bo',...                                        % Blue o
        "LineWidth",2,...                               %
        "MarkerSize",5); hold on                        %
    path_plot = plot(x_path,y_path,'b'); hold on        % Path experienced
    current_goal_plot = line([pt_current(1) PT_GOAL(1)],...
        [pt_current(2) PT_GOAL(2)],...                  % Current-goal line
        'Color','green','LineStyle','--');              %
    circle_plot = drawcircle(pt_current,...             % Sensor range
        SENSOR_RANGE); hold on
    
    [isExist,curve,endpoints] = getcurve(obstacles,...  % Scan with sensor to get
        pt_current,...                          % the intersecting curve
        PT_GOAL,SENSOR_RANGE);                  % and indicate endpoints
    
    if ~isExist             % No blocking obstacle
        pt_dest = PT_GOAL;  % Drive towards goal point
    else                    % Existing blocking obstacle
        if checkIntersect(pt_current,PT_GOAL,endpoints)
            curve_plot = plot(curve(1,:),curve(2,:),'r.'); hold on
            endpt_plot = plot(endpoints(1,:),endpoints(2,:),'ro'); hold on
        end
        pt_dest = decideOi(pt_current,...   % Drive towards Oi
            PT_GOAL,endpoints);             %
    end
    axis([left_bottom(1) right_top(1) left_bottom(2) right_top(2)]);
    axis equal
    
%     disp(['Go to endpoint: (',...
%             num2str(pt_dest(1)),',',num2str(pt_dest(2)),')']);
    if ~mode
        %%  motion-to-goal
        pt_current = straight2next(pt_current,pt_dest,LEN_STEP);
        
        mode = checkalong(pt_previous,...   % Check the condition 
            pt_current,PT_GOAL,...          % to start boundary following.
            curve,LEN_STEP);                % 
        if mode                             % mode from 0 to 1.
            disp('End motion-to-goal, start boundary-following...');
            angle_previous = atan2(pt_dest(2)-pt_current(2),...
                pt_dest(1)-pt_current(1));
        end
    else
        %%  boundary-following   
        [pt_current,angle_next] = boundaryfollow2next(...    
            angle_previous,...                  % Follow the most recent
            pt_current,pt_dest,LEN_STEP,obstacles); % direction. Initially
                                                % from motion-to-goal
        angle_previous = angle_next;            %
                                                % 
        mode = checkoff(pt_current,...          % Check the condition 
            curve,PT_GOAL,SENSOR_RANGE,...      %
            obstacles,LEN_STEP,endpoints);      % to terminate boundary
                                                % following.
        if ~mode                                % mode from 1 to 0.
            disp('End boundary-following, start motion-to-goal...');
        end
    end
    
    x_path = [x_path pt_current(1)];
    y_path = [y_path pt_current(2)];
    
    pause(0.01);
    if pt_current ~= PT_GOAL
        delete(path_plot);  
        delete(robot_plot);
        delete(circle_plot);
        delete(current_goal_plot);
        if isExist && ~isempty(curve_plot)
            delete(curve_plot);
            delete(endpt_plot);
        end
    else
        break;
    end
end

disp('Reach goal point!');
toc