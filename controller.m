function [F, M, trpy, drpy] = controller(qd, t, params)
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn
%       (qn will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
%  params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded

%% =================== Your code goes here ===================
Kp_xyz = [10, 10, 10];
Kd_xyz = [5, 1, 1];

Kp_RPY = [1500, 1500, 1500];
Kd_RPY = [150, 150, 150];

acc_xyz = qd.acc_des - Kd_xyz.*(qd.vel - qd.vel_des) - Kp_xyz.*(qd.pos - qd.pos_des);

phi_des   = (acc_xyz(1)*sin(qd.yaw_des)/params.grav) - (acc_xyz(2)*cos(qd.yaw_des)/params.grav);
theta_des = (acc_xyz(1)*cos(qd.yaw_des)/params.grav) + (acc_xyz(2)*sin(qd.yaw_des)/params.grav);
psi_des   = qd.yaw_des;

u = zeros(4,1);
u(1) = (params.grav + acc_xyz(3))*params.mass;
u(2:4) = params.I * [Kp_RPY(1)*(phi_des - qd.euler(1)) + Kd_RPY(1)*(0 - qd.omega(1));
                     Kp_RPY(2)*(theta_des - qd.euler(2)) + Kd_RPY(2)*(0 - qd.omega(2));
                     Kp_RPY(3)*(psi_des - qd.euler(3)) + Kd_RPY(3)*(qd.yawdot_des - qd.omega(3))]; % control input u, you should fill this in
                  
% Thrust
F = u(1);       % This should be F = u(1) from the project handout

% Moment
M = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

end

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
