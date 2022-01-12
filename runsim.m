%% ***************** QUADROTOR SIMULATION ***************** %
%% Clear Workspace
close all
clear all
clc

addpath('utils')
addpath('trajectories')

OUTPUT_TO_VIDEO = 0;
videoName = 'circle';

%% Reference Trajectory
% Select the Wanted Trajectory

% trajhandle = @hoverTrajectory;
% trajhandle = @circleTrajectory;
trajhandle = @diamondTrajectory;

%% Regulator
controlhandle = @controller;

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal;
grid on;
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
real_time = true;
params = crazyflie();
time_tol = 5000;      % max time
maxIter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err       = []; % runtime errors

% Get start and stop position
startTraj = trajhandle(0);
stopTraj  = trajhandle(inf);
stopPose  = stopTraj.pos;
x0        = initializeState(startTraj.pos, 0);
xtraj     = zeros(maxIter*nstep, length(x0));
ttraj     = zeros(maxIter*nstep, 1);
x         = x0;

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter(videoName);
    open(v);
end

fprintf('Simulation Running....\n')
% Main loop
for iter = 1:maxIter
    tic;

    timeint = time:tstep:time+cstep;
    % Initialize quad plot
    if iter == 1
        QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), maxIter, h_3d);
        desiredState = trajhandle(time);
        QP.UpdateQuadPlot(x, [desiredState.pos; desiredState.vel], time);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end
    
    % Run simulation
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, 1, controlhandle, trajhandle, params), timeint, x);
    x = xsave(end, :)';
    
    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    % Update quad plot
    desiredState = trajhandle(time + cstep);
    QP.UpdateQuadPlot(x, [desiredState.pos; desiredState.vel], time + cstep);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    if OUTPUT_TO_VIDEO == 1
        im = frame2im(getframe(gcf));
        writeVideo(v,im);
    end
    
    time = time + cstep; % Update simulation time
    t = toc;

    % Check to make sure ode45 is not timing out
    if(t > cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminateCheck(x, time, stopPose, pos_tol, vel_tol, time_tol)
        break
    end
end

if OUTPUT_TO_VIDEO == 1
    close(v);
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);


% Plot the saved position and velocity of each robot
% Truncate saved variables
QP.TruncateHist();
% Plot position for each quad
h_pos = figure('Name', ' position');
plotState(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
plotState(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
% Plot velocity for each quad
h_vel = figure('Name', 'velocity');
plotState(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
plotState(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');

if(~isempty(err))
    error(err);
end

fprintf('Finished!\n')
