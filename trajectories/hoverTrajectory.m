function [desiredState] = hoverTrajectory(t)
% Hover Trajectory Generator
T = 3;
height = 5;

if t > T
    pos = [0; 0; height];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
else
    [p, v, a] = positionOverLine(0, height, T, t);
    pos = [0; 0; p];
    vel = [0; 0; v];
    acc = [0; 0; a];
end

yaw = 0;
yawdot = 0;

desiredState.pos = pos(:);
desiredState.vel = vel(:);
desiredState.acc = acc(:);
desiredState.yaw = yaw;
desiredState.yawdot = yawdot;

end
