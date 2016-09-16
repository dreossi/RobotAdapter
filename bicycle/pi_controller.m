function [ u, err_sum ] = pi_controller( ref_val, val, params, err_sum )
%PI_CONTROLLER
    % PI controller
    % INPUT
    % ref_val: reference value
    % val: actual value
    % params: [kp ki] PI constants
    % err_sum_theta: accumulated error
    %
    % OUTPUT
    % u: output command 
    % err_sum: updated accumulated error
    
    kp = params(1,1);
    ki = params(1,2);
       
    err = ref_val - val;
    u = kp*err + ki*err_sum;
    err_sum = err_sum + err;


end

