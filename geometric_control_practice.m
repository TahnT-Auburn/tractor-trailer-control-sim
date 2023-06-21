%% Single Unit Vehicle Lateral Controller Simulation Practice

close all
clear variables
clc

%% GENERATE TRAJECTORY

% load data
data = load('VehicleDynamics_31_12_2002_____20_14_02.mat');

% elimate NaNs from GPS position
gpsPosVect = [data.gSeptentrio.zECEF_X'; data.gSeptentrio.zECEF_Y'; data.gSeptentrio.zECEF_Z'];
count = 1;
for i = 1:length(data.gSeptentrio.zECEF_X)
    if sum(isnan(gpsPosVect(:,i))) == 0
        idx(count) = i;
        count = count+1;
    end
end
% ecef position
ecefPos = gpsPosVect(:,idx);
% lla position
llaPos = ecef2lla(ecefPos', 'WGS84')';
% enu position
enuPos = lla2enu(llaPos', llaPos(:,1)', 'flat')';
enuPos = [enuPos(1,55:end-5); enuPos(2,55:end-5); zeros(1,length(enuPos(1,55:end-5)))];

% lla plot position
% figure
% geoplot(llaPos(1,:), llaPos(2,:));
% geobasemap satellite
% hold on
% geoplot(llaPos(1,1), llaPos(2,1), 'o')
% geoplot(llaPos(1,end), llaPos(2,end), 'o')
% legend('Trajectory', 'Start', 'End')

% lane change trajectory
xLength = 0:150;
lcTrajectory = [-xLength; 3*sin(0.05*xLength); zeros(1,length(xLength))];
% lcTrajectory = [xLength; 3*ones(1,length(xLength)); zeros(1,length(xLength))];

% desired trajectory
desTrajectory = lcTrajectory;

% trajectory length (for external data sets)
for i = 2:length(desTrajectory)
    dist(i) = norm(desTrajectory(:,i) - desTrajectory(:,i-1)); 
end
trajLength = sum(dist);

%% VEHICLE PARAMETERS

% vehicle parameters
a = 1.72;   % CG dist to front axle
b = 1.25;   % CG dist to rear axle
L = a+b;    % wheelbase 

% vehicle velocity
v = 15;

% initialize vehicle states (nav frame)
xinit = desTrajectory(1,1);
yinit = desTrajectory(2,1);
zinit = 0;
yawinit = pi;

%% PURE PURSUIT CONTROLLER
% uses kinematic model (specified reference at front, CG, or rear)

% initialize vehicle states (nav frame)
x = xinit;
y = yinit;
z = zinit;
yaw = yawinit;
xPos = x;
yPos = y;
heading = yawinit;
headingRate = 0;

% look ahead distance
kdd = 0.5;
ld = kdd*v;

T = 0.01;
t_sim = 0:T:trajLength/v;
iter = 1;
for i = 2:length(t_sim)
    
    % vehicle position
    vehPos = [x;y;z];
    
    % search for waypoint
    for k = 2:length(desTrajectory)
        
        if norm(desTrajectory(:,k) - vehPos) >= ld && norm(desTrajectory(:,k-1) - vehPos) < ld
    
            % nav frame waypoint
            waypoint = desTrajectory(:,k);
            wp(:,iter) = waypoint; % track waypoints
            iter = iter+1;

        end
    end

    % position offset
    deltaX(i) = waypoint(1) - x;
    deltaY(i) = waypoint(2) - y;
    deltaZ = 0;
    
    % generate bodyframe RPV
    Cnb = [cos(yaw), sin(yaw), 0; ...   % nav to bodyframe rotation matrix
           -sin(yaw), cos(yaw), 0; ...
            0, 0, 1];
    bodyFrameRpv = Cnb*[deltaX(i); deltaY(i); deltaZ];

    % alpha
    alpha(i) = atan(bodyFrameRpv(2)/bodyFrameRpv(1));
    
    if alpha > pi/2
        alpha = alpha - pi;
    elseif alpha < -pi/2
        alpha = alpha + pi;
    end
    
    % cross-track error
    ctError = sin(alpha)*ld;

    % desired curvature
    K = 2*bodyFrameRpv(2)/ld^2;
    
    % set gain scale
    gainScale = 1;

    % steer angle
%     steerAngle(i) = atan(2*L*sin(alpha(i))/ld);
    steerAngle(i) = atan(gainScale*K*L);
    
    % satrate steer
    steerMax = pi/2;
    steerMin = -steerMax;
    if steerAngle(i) > steerMax
        steerAngle(i) = steerMax;
    elseif steerAngle(i) < steerMin
        steerAngle(i) = steerMin;
    end

    % update kinematics
    yawRate = (v*tan(steerAngle(i)))/L;
    yaw = wrapToPi(yaw + yawRate*T);
    xd = v*cos(yaw);
    yd = v*sin(yaw);
    x = x + xd*T;
    y = y + yd*T;
    z = 0;

    % siphon variables
    headingRate(i) = yawRate;
    heading(i) = yaw;
    xPos(i) = x;
    yPos(i) = y;

end

% find unique waypoints
col = 1;
count = 1;
for i = 2:length(wp)
    if norm(wp(:,i)) ~= norm(wp(:,i-1))
        col(count) = i;
        count = count+1;
    else
        continue
    end
end

% set new waypoints
wp1 = wp(:,col);

% plots
figure
hold on
plot(desTrajectory(1,:), desTrajectory(2,:), '--', LineWidth=1.5)
plot(xPos, yPos, LineWidth=1.5)
plot(wp1(1,:), wp1(2,:), '*')
hold off
title('Global Position')
xlabel('East [m]')
ylabel('North [m]')
legend('Trajectory', 'Vehicle Path', 'Waypoints')
set(gcf, 'color', 'w')

figure
subplot(4,1,1)
plot(t_sim, rad2deg(steerAngle), LineWidth=1.5)
ylabel('Deg')
title('Steer Angle')
subplot(4,1,2)
plot(t_sim, rad2deg(heading), LineWidth=1.5)
ylabel('Deg')
title('Heading')
set(gcf, 'color', 'w')
subplot(4,1,3)
plot(t_sim, ctError, LineWidth=1.5)
title('Cross-track Error')
ylabel('m')
subplot(4,1,4)
plot(t_sim, rad2deg(alpha), LineWidth=1.5)
ylabel('Deg')
xlabel('Time[s]')
title('Alpha')
set(gcf, 'color', 'w')

%% STANLEY CONTROLLER

% reinitialize terms
% waypoint = zeros(3,length(desTrajectory));
% wp = zeros(3,length(desTrajectory));

% initialize vehicle states (nav frame)
x = xinit;
y = yinit;
z = zinit;
yaw = yawinit;
xPos = x;
yPos = y;
heading = yawinit;
headingRate = 0;

T = 0.01;
t_sim = 0:T:trajLength/v;
iter = 1;
for i = 1:length(t_sim)

    % vehicle position
    vehPos = [x;y;z];
     
%     % search for waypoint
%     for k = 2:length(desTrajectory)-1
%         
%         if norm(desTrajectory(:,k) - vehPos) >= ld && norm(desTrajectory(:,k-1) - vehPos) < ld
%     
%             % nav frame waypoint
%             waypoint = desTrajectory(:,k);
%             wp(:,iter) = waypoint; % track waypoints
%             iter = iter+1;
% 
%             % tangent slope approximation
%             m = (desTrajectory(2,k+1) - desTrajectory(2,k-1))/(desTrajectory(1,k+1) - desTrajectory(1,k-1));
% 
%         end
%     end
    
    % search for closest waypoint
    dist = zeros(1,length(desTrajectory));
    for k = 1:length(desTrajectory)
        
        dist(k) = norm(desTrajectory(:,k) - vehPos);

    end
    [mini, index] = min(dist);
    
    if index == 1
        index = index+1;
    elseif index == length(desTrajectory)
        index = index-1;
    end
    
    % set waypoint
    waypoint = desTrajectory(:,index);
    wp(:,i) = waypoint; % track waypoints

    % tangent slope approximation
    m(i) = (desTrajectory(2,index+1) - desTrajectory(2,index))/...
         abs((desTrajectory(1,index+1) - desTrajectory(1,index)));
    
    % global trajectory heading
%     psiTrajectory(i) = atan(deltY(i)/deltX);
%     if sign(psiTrajectory(i)) ~= sign(yaw) 
%         psiTrajectory(i) = pi - psiTrajectory(i);
%     end
    
    % global frame trajectory heading
    psiTrajectory(i) = atan2(m(i),1);

    % re-orient trajectory heading 
    if (desTrajectory(1,index+1) - desTrajectory(1,index)) < 0 && m(i) > 0
        psiTrajectory(i) = pi/2 + psiTrajectory(i);
    elseif (desTrajectory(1,index+1) - desTrajectory(1,index)) < 0 && m(i) < 0
        psiTrajectory(i) = -pi/2 + psiTrajectory(i);
    end

    % heading error
    psiError(i) = psiTrajectory(i) - yaw;
    
    % position offset
    deltaX(i) = waypoint(1) - x;
    deltaY(i) = waypoint(2) - y;
    deltaZ = 0;
    
    % generate bodyframe RPV
    Cnb = [cos(yaw), sin(yaw), 0; ...   % nav to bodyframe rotation matrix
           -sin(yaw), cos(yaw), 0; ...
            0, 0, 1]; 
    bodyFrameRpv = Cnb*[deltaX(i); deltaY(i); deltaZ];

    crossTrackError(i) = bodyFrameRpv(2);

%     % search minimum distance between vehicle and trajectory
%     for j = 1:length(desTrajectory)
%     
%         dist(j) = norm(desTrajectory(:,j) - vehPos);
%   
%     end
%     % cross-track error
%     [crossTrackError(i), index] = min(dist);
%     
%     if index > 1
%         % tangent slope aprroximation
%         m = (desTrajectory(2,index+1) - desTrajectory(2,index-1))/(desTrajectory(1,index+1) - desTrajectory(1,index-1));
%     else
%         m = 0;
%     end
% 
%     % generate global trajectory heading
%     deltX = 1;
%     deltY = m*deltX;
%     psiTrajectory(i) = atan(deltY/deltX);
%     
%     % heading error
%     psiError(i) = psiTrajectory(i) - yaw;

    % portional gain
    Kp = 1;

    % steer angle
    steerAngle(i) = psiError(i) + atan(Kp*crossTrackError(i)/v);
    
    % saturate steer
    steerMax = pi/2;
    steerMin = -steerMax;
    if steerAngle(i) > steerMax
        steerAngle(i) = steerMax;
    elseif steerAngle(i) < steerMin
        steerAngle(i) = steerMin;
    end

    % update kinematics
    yawRate = (v*tan(steerAngle(i)))/L;
    yaw = wrapToPi(yaw + yawRate*T);
    xd = v*cos(steerAngle(i) + yaw);
    yd = v*sin(steerAngle(i) + yaw);
    x = x + xd*T;
    y = y + yd*T;
    z = 0;

    % siphon variables
    headingRate(i) = yawRate;
    heading(i) = yaw;
    xPos(i) = x;
    yPos(i) = y;
end

% find unique waypoints
col = 1;
count = 1;
for i = 2:length(wp)
    if norm(wp(:,i)) ~= norm(wp(:,i-1))
        col(count) = i;
        count = count+1;
    else
        continue
    end
end

% plots
figure
hold on
plot(desTrajectory(1,:), desTrajectory(2,:), '--', LineWidth=1.5)
plot(xPos, yPos, LineWidth=1.5)
plot(wp(1,:), wp(2,:), '*')
hold off
title('Global Position')
xlabel('East [m]')
ylabel('North [m]')
legend('Trajectory', 'Vehicle Path', 'Waypoints')
set(gcf, 'color', 'w')

figure
subplot(4,1,1)
plot(t_sim, rad2deg(steerAngle), LineWidth=1.5)
title('Steer Angle')
ylabel('Deg')
subplot(4,1,2)
plot(t_sim, rad2deg(heading), LineWidth=1.5)
title('Heading')
ylabel('Deg')

subplot(4,1,3)
plot(t_sim, rad2deg(psiError), LineWidth=1.5)
title('Heading Error')
ylabel('Deg')
subplot(4,1,4)
plot(t_sim, crossTrackError, LineWidth=1.5)
title('Cross Track Error')
ylabel('m')
xlabel('Time [s]')
set(gcf, 'color', 'w')



%% INDY CAR VEHICLE PARAMETERS

% a = 1.72;   % CG dist to front axle
% b = 1.25;   % CG dist to rear axle
% Cf = 143e3; % front cornering stiffness
% Cr = 143e3; % rear cornering stiffness
% Iz = 1200;  % yaw mass moment of inertia
% m = 720;    % vehicle mass
% N = 12;     % steering column gear ratio 
% 
% c0 = Cf + Cr;
% c1 = a*Cf - b*Cr;
% c2 = (a^2)*Cf + (b^2)*Cr;
% 
% % vehicle velocity
% V = 15;
% 
% %% BICYLCE MODEL OPEN LOOP TF
% 
% numOpen = [a*Cf,  (a*Cf - c1*Cf)/(m*V)];
% denOpen = [Iz, (c0*Iz + m*c2)/(m*V), (c0*c2 - c1*m*V^2 - c1^2)/(m*V^2)];
% tfOpen = tf(numOpen,denOpen);
% 
% % simulate open loop model
% dt = 0.01;
% t_sim = 0:dt:10;
% steerAngle = deg2rad(10);
% u = steerAngle*ones(1,length(t_sim));
% y = lsim(tfOpen, u ,t_sim);