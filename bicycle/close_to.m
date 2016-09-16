function [ in ] = close_to( p1,p2,eps )
%CLOSE_TO
    % Checck if p1 is close to p2 by eps
    % INPUT
    % p1: [x y] point 1
    % p2: [x y] point 2
    % eps: tolerance
    
    in = (p2(1)-eps <= p1(1)) && (p1(1) <= p2(1)+eps) && (p2(2)-eps <= p1(2)) && (p1(2) <= p2(2)+eps);
end

