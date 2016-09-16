function [ P_i_up ] = adapt( )
%OPTIMIZE
%   Optimization function based on gradient descent
    clear
    close all

    % Set initial conditions for simulation
    simulation.X0 = [0;0;pi/4;0;0.25;0];
    simulation.way_pts = [0 0 ; 50 0; 100 0; 150 0 ; 200 0];
    %(2)%simulation.way_pts = [0 0 ; 100 40 ; -40 60; -100 100; 60 100; -20 40 ; -60 20; 0 0];
    %(1)%simulation.way_pts = [0 0 ; -30 40 ; 30 90; 0 140];
    simulation.Delta = 0.01;    % Discretization step
    %simulation.P = [1 ; 1];     % Initial parameters  
    %simulation.P = [2 ; 2; 0.0015; 0.5];
    simulation.P = [0.5 ; 0.0001];
    
    % Optimization parameters
    %P_eps = [0.001;0.001;0; 0.00001];    % change in parameters for estimate partial derivatives
    P_eps = [0.001;0.00001];    % change in parameters for estimate partial derivatives
    
    %alphas = [0.00005; 0.00005; 0.000000005; 0.000005];    % learning rates for gradient descent
    alphas = [0.00005; 0.000000005];    % learning rates for gradient descent
    MAX_ITER = 30;    % Maximum number of iterations
    
    
    % Breach initialization
    system.name = 'Brass';
    system.init_state = simulation.X0';
    options.phi = 'alw_[0, 400] ( (x2[t] < 1) or (ev_[0, 50] (x2[t] < 0.2)))';
    
    % Initialize Breach objects necessary to compute robustness
    breachMonitor =  init_breach_monitor(system,options);
    
    % Plot way points
    figure(1)
%    subplot(1,2,1)
    hold on
    plot(simulation.way_pts(:,1),simulation.way_pts(:,2),'*r');
    xlabel('x');
    ylabel('y'); 
    
    figure(2)
    xlabel('t');
    ylabel('v'); 
    
    for i=1:MAX_ITER
        
        % Compute objective function value with current parameters
        P_i = simulation.P;
        val_i = obj_fun(simulate(simulation),breachMonitor);
        
        % Alter current parameters (used for partial derivatives estimation)
        simulation.P = P_i + P_eps;
        
        % Apply gradient descent on each parameters
        P_i_up = P_i;
        for j=1:size(simulation.P,1)            
            val_i_eps = obj_fun(simulate(simulation),breachMonitor); 
            P_i_up(j,1) = gradient_descent([val_i val_i_eps],[P_i(j,1) simulation.P(j,1)], alphas(j) );
        end        
        
        % Update new parameters
        simulation.P = P_i_up;
        P_i_up
        
        [traj,simulation] = simulate(simulation);
        obj_fun(traj,breachMonitor)
        
        
        % Plot trajectory 
        figure(1)
        %subplot(1,2,1)
        hold on
        plot(traj.X(1,:),traj.X(2,:),'b');
        drawnow
        
%         subplot(1,2,2)
%         plot(simulation.way_pts(:,1),simulation.way_pts(:,2),'*--r');
%         hold on
%         for j=1:size(simulation.way_pts,1)-1
%             text(simulation.way_pts(j,1)+1,simulation.way_pts(j,2),num2str(simulation.vs_steer(j)));
%         end
%         axis([-200 200 0 120]);
%         hold off
        
        figure(2)
        hold on
        plot(traj.time,traj.X(5,:),'b');
        drawnow      
        
    end
    
   
    % Plot final trajectories
    figure(1)
    %subplot(1,2,1)
    hold on
    for i=2:size(traj.X,2)
        %if( traj.X(5,i) > traj.X(5,i-1) )
        %    col = 'g';
        %else
        %    if( traj.X(5,i) == traj.X(5,i-1) )
        %        col = 'k';
        %    else
                col = 'r';
        %    end
        %end
        plot(traj.X(1,i),traj.X(2,i),['.' col]);
    end    
    figure(2)
    hold on
    plot(traj.time,traj.X(5,:),'r','LineWidth', 2); 
end

% Initialize Breach objects necessary to compute robustness
%
% Assumes that 
%     - options has a field phi, which is a string describing an STL formula to falsify
%     - the system signals have names 'x_1' to 'x_N' where N is the dimension of system.init_state
%  
function [breachMonitor] = init_breach_monitor(system, options)  

    if isfield(options, 'phi')
        nb_signals = numel(system.init_state);
        breachMonitor.Sys = CreateExternSystem(system.name, nb_signals, 0);
        breachMonitor.P = CreateParamSet(breachMonitor.Sys);
        breachMonitor.traj.time = [];
        breachMonitor.traj.param = zeros(1, nb_signals);
        breachMonitor.traj.X = zeros(nb_signals, 0);
        if ischar(options.phi)
            breachMonitor.phi = STL_Formula('phi', options.phi);%QMITL_Formula('phi', options.phi);
        else
            breachMonitor.phi = options.phi;
        end
    else
        breachMonitor.Sys = [];
        breachMonitor.phi = [];
        breachMonitor.traj = [];
        breachMonitor.P=[]; 
    end
end

