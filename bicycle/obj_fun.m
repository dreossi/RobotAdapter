function [ val ] = obj_fun( traj, breachMonitor )
%OBJ_FUN objectvie funtion to be optimized
    %val = traj.time(end); 
    
    [val,~] = STL_EvalClassic(breachMonitor.Sys,breachMonitor.phi,breachMonitor.P,traj,0);
    %val = traj.time(end); 
    
    
%         if(max(traj.X(2,:)) > 113)
%             val = 10000000;
%         else
%             val = traj.time(end);
%         end

    
end

