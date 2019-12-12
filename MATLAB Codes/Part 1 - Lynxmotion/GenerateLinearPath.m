function [pts,t_interval,vee] = GenerateLinearPath(pt_start,pt_end,u,M,motionFlag)
%   Generate linear path between start point and end point.
%   IMPORTANT: The path excludes start point and end point.
%   MotionFlag: 1 constant | 0 varying speed
if nargin == 4
    motionFlag = 0;
end

vee = ones(1,M+2);
delta_dist = pt_end-pt_start;
dist = norm(delta_dist);
% Constant speed
if motionFlag
    pts = [linspace(pt_start(1),pt_end(1),M+2);...
        linspace(pt_start(2),pt_end(2),M+2);...
        linspace(pt_start(3),pt_end(3),M+2)];
    t_total = dist/u;
    t_interval = t_total/(M+1);
    vee = vee*u;
    
% Varying accelerated motion
else
    % motion parameters
    t2_t1_ratio = 2;
    t2 = dist/u;
    t1 = t2/t2_t1_ratio;
    t_total = (1 + t2_t1_ratio)*t1;   % positive acceleration duration time = 1/3*
    t_interval = t_total/(M+1); % constant motion duration time
    a = t2_t1_ratio*(u^2)/dist; % acceleration rate
    % ratio on three axes
    delta_ratio = [delta_dist(1);...
        delta_dist(2);...
        delta_dist(3)]/dist;
    pts = zeros(3,M+2);
    for i = 0:M+1   % M sample points -> M+1 intervals
        t = i*t_interval;
        if t <= t1
            pts(:,i+1) = pt_start + 1/2*a*(t^2)*delta_ratio;
            vee(:,i+1) = a*t;
        elseif t > t1 && t <= t2
            pts(:,i+1) = pt_start + (1/2*a*(t1^2) + u*(t-t1))*delta_ratio;
            vee(:,i+1) = u;
        else
            pts(:,i+1) = pt_start + (dist - 1/2*a*((t_total-t)^2))*delta_ratio;
            vee(:,i+1) = u - a*(t-t2);
        end
    end
end

%   Exclude start point and end point
pts = pts(:,2:M+1);
vee = vee(:,2:M+1);
end