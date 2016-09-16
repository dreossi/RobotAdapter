function [ x1p ] = gradient_descent( Jx, x, alpha )
%GRADIENT_DESCENT Single step of gradient descent
    %
    % Jx: [Jx1 Jx2] two evaluations of J on x1 and x2
    % x: [x1 x2] x1 and x2
    % alpha: learning rate 
        
    x1p = x(1,1) + alpha*dFdx(Jx,x);
    
end

