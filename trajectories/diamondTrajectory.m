function [desiredState] = diamondTrajectory(t)
% Diamond Trajectory Generator

T = 10;
dt = 0.0001;

    function pos = getPosition(t)
        x = positionOverLine(0, 1, T, t);
        if t < (1/4)*T
            y = positionOverLine(0, sqrt(2), T/4, t);
            z = positionOverLine(0, sqrt(2), T/4, t);
            
        elseif t < 2*(1/4)*T
            y = positionOverLine(sqrt(2), 0, T/4, t-T/4);
            z = positionOverLine(sqrt(2), 2*sqrt(2), T/4, t-T/4);

        elseif t < 3*(1/4)*T
            y = positionOverLine(0, -sqrt(2), T/4, t-2*T/4);
            z = positionOverLine(2*sqrt(2), sqrt(2), T/4, t-2*T/4);

        else 
            y = positionOverLine(-sqrt(2), 0, T/4, t-3*T/4);
            z = positionOverLine(sqrt(2), 0, T/4, t-3*T/4);
        end
        pos = [x; y; z];
        
    end

    function vel = getVelocity(t)
        pos1 = getPosition(t);
        pos2 = getPosition(t+dt);
        vel = (pos2 - pos1)/dt;
    end

if t > T
    pos = [1; 0; 0];
    vel = [0;0;0];
    acc = [0;0;0];
else
    pos = getPosition(t);
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
