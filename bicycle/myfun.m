function [ val ] = myfun( p )
%MYFUN 

    simulation.X0 = [0;0;0;0;0;0];
    simulation.way_pts = [0 0 ; 100 40 ; -40 60; -100 100; 60 100; -20 40 ; -60 20; 0 0];
    simulation.Delta = 0.01;    % Discretization step
    %simulation.P = [1 ; 1];     % Initial parameters  
    simulation.P = [1.1000; 1.1000; 0.0015; p];

    val = obj_fun(simulate(simulation));


end

