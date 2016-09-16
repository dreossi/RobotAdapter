function [ traj, simulation ] = simulate( simulation )
%SIMULATE 
    % Simulate the bicycle system
    %
    % INPUT
    % simulation.X0: initial state (see bycycle.m for state descritpion)
    % simulation.way_pts: [p1_x p1_y ; p2_x p2_y ; ... ] list of waypoints to reach
    % simulation.Delta: simulation discretization step
    % simulation.P: parameters
    %
    % OUTPUT
    % traj.X; computed trajecotry
    % treaj.time: timestamps
    
    % Maximum time-out
    MAX_TIME = 1000; 
    
    % Get input
    X0 = simulation.X0;
    way_pts = simulation.way_pts;
    Delta = simulation.Delta;
    P = simulation.P; 
    
    % Compute steering velocity and breaking distances (for reference velocities)
    %[vs_steer, break_ds] = gen_v_steer(simulation.way_pts,P(4,1));
    [vs_steer, break_ds] = gen_v_steer(simulation.way_pts,1);
    simulation.vs_steer = [0 vs_steer];
    simulation.break_ds = [0 break_ds];
        
    % Initialize PID error accumulators
    err_sum = 0;
    err_sum_theta = 0;   

    % Initialize trajectory
    traj.X = X0; 
    traj.time = 0; 
    goal_pt_i = 1;
    goal_pt = way_pts(goal_pt_i,:); 
    v_steer = simulation.vs_steer(goal_pt_i);
    end_simulation = 0;

    i = 1;
    while ~(end_simulation)

        % Check if goal point has been reached
        if( close_to(traj.X(1:2,i)',goal_pt,1) )
            goal_pt_i = goal_pt_i + 1;
            % Check if last goal point has been reached
            if(goal_pt_i > size(way_pts,1))
                break;
            else
                goal_pt = way_pts(goal_pt_i,:);  
                %if(goal_pt_i>length(simulation.vs_steer))
                %    v_steer = 0;
                %    break_dist = 5;
                %else
                    v_steer = simulation.vs_steer(goal_pt_i);
                    break_dist = simulation.break_ds(goal_pt_i);
                %end
                
                
            end
        end

        % Get actual pose
        x = traj.X(1,i);
        y = traj.X(2,i);
        theta = traj.X(3,i);
        vel = traj.X(5,i);

        % Compute heading and velocity references values
        theta_ref = atan2(goal_pt(2)-y,goal_pt(1)-x);
        dist_next_wp = sqrt( (x-goal_pt(1))^2 + (y-goal_pt(2))^2 );
        v_ref = gen_v_ref(dist_next_wp,v_steer,break_dist);
        % Call PI controllers (for steering and velocity)
        %[U(1,i), err_sum_theta] = pi_controller(theta_ref, theta, [P(1,1) 0], err_sum_theta);           
        [U(1,i), err_sum_theta] = pi_controller(theta_ref, theta, [P(1,1) P(2,1)], err_sum_theta);           
        %[U(2,i), err_sum] = pi_controller(v_ref,traj.X(5,i),[P(2,1) 0.0001],err_sum);
        %[U(2,i), err_sum] = pi_controller(v_ref,traj.X(5,i),[P(2,1) P(3,1)],err_sum);        
        %[U(2,i), err_sum] = pi_controller(v_ref,traj.X(5,i),[0.1 0.001],err_sum);
        U(2,i) = 0;

        % Get new state
        traj.X(:,i+1) = bicycle( traj.X(:,i), U(:,i), Delta );
        traj.time(i+1) = i*Delta;

        i = i + 1;
        end_simulation = (traj.time(i) >= MAX_TIME);        
    end  
    
    

end

