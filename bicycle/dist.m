function [ d ] = dist( x,y )
%Dist Euclidean distance between x and y
    d = sqrt( (x(1)-y(1))^2 + (x(2)-y(2))^2 );
end

