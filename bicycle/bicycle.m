function [ XP ] = bicycle( X, U, Delta )
%BYCYCLE 
    % Dynamics of a bycycle veichle model
    %
    % STATE VARIABLES
    % x,y: position in plane (center of rear axle)
    % v,a: speed and acceleration (at the rear axle)
    % theta: car heading
    % delta: steering angle
    %
    % INPUT VARIABLES
    % delta_c: steering command
    % a_c: longitudinal acceleration
    %
    % PARAMETERS
    % T_d, T_a: first-order lag time constants for steering and
    % acceleration
    % delta_max: maximum steering angle
    % a_min, a_max: minimum and maximum acceleration
    % vCH: characteristic velocity (for side slip)
    % L: wheelbase (distance between wheels)
    
    % Set parameters
    T_d = 0.05; T_a = 0.3;
    delta_max = 0.5435;
    a_min = -6.0; a_max = 1.8;    
    L = 2.885;
    
    % Get state
    x = X(1,1); y = X(2,1);
    theta = X(3,1); delta = X(4,1);
    v = X(5,1); a = X(6,1);
    vCH = 50.0;
    
    % Get input
    delta_c = U(1,1); a_c = U(2,1);    
    
    % Impose physical limits on steering input
    if (delta_c < -delta_max)
        delta_c = -delta_max;
    end
    if (delta_c > delta_max)
        delta_c = delta_max;
    end
    % Impose physical limits on acceleration input
    if (a_c < a_min)
        a_c = a_min;
    end
    if (a_c > a_max)
        a_c = a_max;
    end
    
    % Side slip
    G_ss = 1/(1+(v/vCH)^2);
    
    xp = x + (v*cos(theta))*Delta;
    yp = y + (v*sin(theta))*Delta;
    %thetap = theta + ((v/L)*tan(delta)*G_ss)*Delta;
    thetap = theta + ((v/L)*(delta)*G_ss)*Delta;
    deltap = delta + ((1/T_d)*(delta_c - delta))*Delta;
    vp = v + (a)*Delta;
    ap = a + ((1/T_a)*(a_c-a))*Delta;
    
    XP = [xp; yp; thetap; deltap; vp; ap];

end

