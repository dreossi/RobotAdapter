function [  ] = test( x )
%OPTIMIZE
%   Optimization function based on gradient descent
    close all

    % Set initial conditions for simulation
    simulation.X0 = [0;0;0;0;0;0];
    simulation.way_pts = [0 0 ; 100 40 ; -40 60; -100 100; 60 100; -20 40 ; -60 20; 0 0];
    simulation.Delta = 0.01;    % Discretization step
    %simulation.P = [1 ; 1];     % Initial parameters  
    simulation.P = x;
        
    [traj,simulation] = simulate(simulation);
    
    traj.time(end)
    hold on
    plot(traj.X(1,:),traj.X(2,:),'b-');
    plot(simulation.way_pts(:,1),simulation.way_pts(:,2),'r*')
    plot([-150:1:150],ones(1,301)*114,'--r')
    
  
        
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

