function [s] = initializeState(start, yaw)
% Initialize 13x1 state vector
 
s    = zeros(13,1);
Rot  = RPYtoRot_ZXY(0.0, 0.0, yaw);
Quat = RotToQuat(Rot);

s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = 0;        %xdot
s(5)  = 0;        %ydot
s(6)  = 0;        %zdot
s(7)  = Quat(1);  %qw
s(8)  = Quat(2);  %qx
s(9)  = Quat(3);  %qy
s(10) = Quat(4);  %qz
s(11) = 0;        %p
s(12) = 0;        %q
s(13) = 0;        %r

end