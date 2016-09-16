function [ vs_steer, break_ds ] = gen_v_steer( waypts, P )
%GEN_V_STEER
    % Get the velocity and breaking distances
    % to curve as function of the angle between way points 
    % P(1) is the gain for the steering curve
    
    V_MAX = 25; % To be fixed
    A_MIN = -6; % Miniumum acceleration
    
    vs_steer = zeros(1,size(waypts,1)-2);
    break_ds = zeros(1,size(waypts,1)-2);
    
    % Generate minimum speeds for curves
    for i=1:size(waypts,1)-2
    
        wp1 = waypts(i+2,:)-waypts(i+1,:);
        wp2 = waypts(i,:)-waypts(i+1,:);
        alpha = abs(atan2(wp2(2),wp2(1)) - atan2(wp1(2),wp1(1)));
    
        % Normalize angle between [0,pi]
        if (alpha > pi)
            alpha = 2*pi - alpha;
        end    
    
        vs_steer(i) = (V_MAX/pi*alpha)*P(1);
        break_ds(i) = abs((V_MAX^2 - vs_steer(i)^2)/(2*A_MIN));
    end
    
    vs_steer(end+1) = V_MAX;
    break_ds(end+1) = 0;
    
end
