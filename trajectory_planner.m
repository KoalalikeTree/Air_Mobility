function trajectory_state = trajectory_planner(question, waypoints, max_iter, waypoint_times, time_step)

% Input parameters
% 
%   question: Which question we are on in the assignment
%
%   waypoints: Series of points in [x; y; z; yaw] format
%
%   max_iter: Number of time steps
%
%   waypoint_times: Time we should be at each waypoint
%
%   time_step: Length of one time_step
%
% Output parameters
%
%   trajectory_sate: [15 x max_iter] output trajectory as a matrix of states:
%   [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
%
%************  TRAJECTORY PLANNER ************************

% Write code here
if question == 2
    % Sample code for hover trajectory
    trajectory_state = zeros(15,max_iter);
    trajectory_state(1,1:400) = 0;
    trajectory_state(1,400:800) = 0.1;
    trajectory_state(1,800:1200) = 0.2;
    trajectory_state(1,1200:end) = 0.3;
    % height of 15 for: [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

    % generate the trajectory automatically
    waypoints = zeros(3,2000);
    waypoints(1,:) = trajectory_state(1, 1:2000);
    waypoints(2,:) = 0;
    waypoints(3, :) = 0.5;
    waypoints = waypoints';

    timeInterval = (0:1:1999);
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',timeInterval, ...
        'SampleRate',1);
end

if question == 3
    % Sample code for hover trajectory
    trajectory_state = zeros(15,max_iter);
    trajectory_state(3,1:400) = 0;
    trajectory_state(3,400:800) = 1;
    trajectory_state(3,800:1200) = 1;
    trajectory_state(3,1200:end) = 0;
    % height of 15 for: [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

    % generate the trajectory automatically
    waypoints = zeros(3,2000);
    waypoints(1,:) = 0;
    waypoints(2,:) = 0;
    waypoints(3, :) = trajectory_state(3, 1:2000);
    waypoints = waypoints';

    timeInterval = (0:1:1999);
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',timeInterval, ...
        'SampleRate',1);
%     % generate the trajectory automatically
%     waypoints = waypoints(1:3, :)';
%     trajectory = waypointTrajectory(waypoints, ...
%         'TimeOfArrival',waypoint_times, ...
%         'SampleRate',50);
end

if question == 4
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',200);
end

if question == 5
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',300);
end

if question == 5.5
    waypoints_xyz = waypoints(1:3, :)';
    
    % get the input yaw angle
    yaw = waypoints(4,:);
    
    % get the column number of the way points and create a 3by3byN matrix
    waypoints_num = size(waypoints);
    waypoints_num = waypoints_num(2);
    oriMatrix = zeros(3, 3, waypoints_num);
    
    % calculate the corresponding rotation matrix with respect to yaw
    for i =  1:waypoints_num
        orientation = [0 0 yaw(i)];
        oriMatrix(:, :, i) = eul2rotm(orientation, 'XYZ');
    end
    
    trajectory = waypointTrajectory(waypoints_xyz, ...
        'TimeOfArrival',waypoint_times, ...
        'Orientation', oriMatrix, ...
        'SampleRate',300);
end

if question == 7.1
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',1000);
end

if question == 7.2
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',800);
end

if question == 7.3
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',600);
end

if question == 7.4
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',400);
end

if question == 7.5
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',200);
end

if question == 7.6
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',100);
end

if question == 7.7
    waypoints = waypoints(1:3, :)';
    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival',waypoint_times, ...
        'SampleRate',50);
end

if question == 8
    % generate the velocity
    vel_xyz = zeros(3, 2001);
    
    time_initial = 0; 
    time_final = 10;
    time_step = 0.005; 
    time_vec = time_initial:time_step:time_final;
%     vel_xyz(1,:) = sin((2*pi/10)*time_vec);
%     vel_xyz(2,:) = cos((2*pi/10)*time_vec);
    vel_xyz(1,:) = 2 * sin((2*pi/10)*time_vec);
    vel_xyz(2,:) = 2 * cos((2*pi/10)*time_vec);
    
%     for i = 1:100
%         vel_xyz(:,i) = i * 0.01 * vel_xyz(:,i);
%     end
    
    % calculate the velocity in x/y direction in each point
%     for i = 1:2001
%         x = waypoints(1, i);
%         y = waypoints(2, i);
%         theta = pi/2 - abs(deg2rad(atand(y/x)));
%         
%         % 第一象限
%         
%         y_vel = double(y_vel);
%         x_vel = double(x_vel);
%         vel_xyz(1, i) = x_vel;
%         vel_xyz(2, i) = y_vel;
%     end
%     vel_xyz = vel_xyz';
    % vel_xyz = zeros(2001, 3);
    % get the angle of the current position
    
    % set trajectory state manually
    trajectory_state = zeros(15, 2001);
    trajectory_state(1:3, :) = waypoints(1:3, :); % xyz
    trajectory_state(4:6, :) = vel_xyz; % xyz dot
    
    %acceleration
    acc = trajectory_state(4:5,2:2001) - trajectory_state(4:5,1:2000);
    trajectory_state(13:14, :) = [acc, zeros(2,1)];
end

if question == 9
     % generate the velocity
    vel_xyz = zeros(3, 2001);
    
    time_initial = 0; 
    time_final = 10;
    time_step = 0.005; 
    time_vec = time_initial:time_step:time_final;
    vel_xyz(1,:) = sin((2*pi/10)*time_vec);
    vel_xyz(2,:) = cos((2*pi/10)*time_vec);

    y_tan = abs(abs(waypoints(2, :))-1);
    x_tan = abs(waypoints(1, :));
    alpha = atand(y_tan./x_tan); % (1by2001)
    yaw = zeros(1, 2001);
    
    for i = 1:2001
        if waypoints(1, i)>=0 && waypoints(2, i)>=1
            % the first quadrant
            yaw(i) = deg2rad(alpha(i));
        end
        if waypoints(1, i)<0 && waypoints(2, i)>=1
            % the second quadrant
            yaw(i) = pi - deg2rad(alpha(i));
        end
        if waypoints(1, i)<0 && waypoints(2, i)<1
            % the third quadrant
            yaw(i) = pi + deg2rad(alpha(i));
        end
        if waypoints(1, i)>=0 && waypoints(2, i)<1
            % the fourth quadrant
            yaw(i) = 2*pi - deg2rad(alpha(i));
        end
    end
    yaw(2001) = yaw(2000);
    
    % set the position and velocity
    trajectory_state = zeros(15, 2001);
    trajectory_state(1:3, :) = waypoints(1:3, :); % xyz
    trajectory_state(4:6, :) = vel_xyz; % xyz dot
    trajectory_state(9, :) = yaw; % psi
end

if (question ~= 8) && (question ~=9)
    for iter = 1:2000
        [position, orientation, velocity, acceleration, angularVelocity] = trajectory();
        trajectory_state(1:3, iter) = position;
        trajectory_state(4:6, iter) = velocity;
        if question == 5.5
            trajectory_state(7:9, iter) = rotm2eul(orientation,'XYZ');
        else
            trajectory_state(7:9, iter) = quat2eul(orientation,'XYZ');
        end
        trajectory_state(10:12, iter) = angularVelocity;
        trajectory_state(13:15, iter) = acceleration;
    end
end

current_waypoint_number = 1;

% % generate the trajectory automatically
% waypoints = waypoints(1:3,:)';
% SampleRate = 1 / time_step;
% trajectory = waypointTrajectory(waypoints, ...
%     'TimeOfArrival',waypoint_times, ...
%     'SampleRate',SampleRate);
% 
% 
% for iter = 1:max_iter
%     [position, orientation, velocity, acceleration, angularVelocity] = trajectory();
%     trajectory_state(1:3, iter) = position;
%     trajectory_state(4:6, iter) = velocity;
%     trajectory_state(7:9, iter) = quat2eul(orientation,'XYZ');
%     trajectory_state(10:12, iter) = angularVelocity;
%     trajectory_state(13:15, iter) = acceleration;
% end

% current_waypoint_number = 1;

% for iter = 1:max_iter
%     %如果现在的waypoint编号小于整体waypoint的长度
%     if (current_waypoint_number<length(waypoint_times))
%         if((iter*time_step)>waypoint_times(current_waypoint_number+1))
%             current_waypoint_number = current_waypoint_number + 1;
%         end
%     end
%         
%     trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number);
%     trajectory_state(9,iter) = waypoints(4,current_waypoint_number);
%     
% end

end
