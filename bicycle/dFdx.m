function [ der ] = dFdx( Fx, x )
%DFDX Approximate the partial derivative of F
    %
    %   INPUT
    %   Fx : [Fx1 Fx2] evaluations of F in x1 and x2
    %   x : [x1 x2] x1 and x2
    
    % Get input
    Fx1 = Fx(1,1);
    Fx2 = Fx(1,2);
    x1 = x(1,1);
    x2 = x(1,2);

    der =  (Fx1 - Fx2)/(x1-x2);
end

