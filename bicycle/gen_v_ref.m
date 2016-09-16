function [ v_ref ] = gen_v_ref( dist_next_wp,v_steer,break_dist )
%GEN_V_REF
    % Generate the actual reference speed 
    
    V_MAX = 5;
    
    if( dist_next_wp < break_dist )
        v_ref = v_steer;
    else
        v_ref = V_MAX;
    end

end
