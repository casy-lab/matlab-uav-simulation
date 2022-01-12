function [pos, vel, acc] = positionOverLine(startPosition, endPosition, totalTime, actualTime)
    h = endPosition - startPosition;
    T = totalTime;
    t = actualTime;
    
    A0 = startPosition;
    A3 = 20*h/(2*T^3);
    A4 = -30*h/(2*T^4);
    A5 = 12*h/(2*T^5);
    
    pos = A0 + A3*t^3 + A4*t^4 + A5*t^5;
    vel = 3*A3*t^2 + 4*A4*t^3 + 5*A5*t^4;
    acc = 6*A3*t + 12*A4*t^2 + 20*A5*t^3;
end