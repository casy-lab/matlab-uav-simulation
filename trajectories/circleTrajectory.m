function [desiredState] = circleTrajectory(t)
% Circle Trajectory Generator
% Set the pos, vel, acc, yaw and yawdot variables
% % NOTE: The simulator will spawn the robot to be at the
% %       position you return for t == 0
%
% pos = [0; 0; 0];
% vel = [0; 0; 0];
% acc = [0; 0; 0];
% yaw = 0;
% yawdot = 0;

T = 10;
radius = 5;
dt = 0.0001;

    function pos = positionFromAngle(a)
        pos = [radius*cos(a); radius*sin(a); 2.5*a/(2*pi)];
    end
    
    function vel = getVelocity(t)
        angle1 = positionOverLine(0, 2*pi, T, t);
        pos1 = positionFromAngle(angle1);
        angle2 = positionOverLine(0, 2*pi, T, t+dt);
        vel = (positionFromAngle(angle2) - pos1)/dt;
    end

if t > T
    pos = [radius; 0; 2.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
else
    angle = positionOverLine(0, 2*pi, T, t);
    pos = positionFromAngle(angle);
    vel = getVelocity(t);
    acc = (getVelocity(t+dt) - getVelocity(t))/dt;
end

yaw = 0;
yawdot = 0;

desiredState.pos = pos(:);
desiredState.vel = vel(:);
desiredState.acc = acc(:);
desiredState.yaw = yaw;
desiredState.yawdot = yawdot;
end
