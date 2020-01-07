function [waypoints, waypoint_times] = lookup_waypoints(question)
    %
    % Input parameters
    %
    %   question: which question of the project we are on 
    %      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
    %
    % Output parameters
    %
    %   waypoints: of the form [x; y; z; yaw]
    % 
    %   waypoint_times: [1 x n] vector of times where n is the number of waypoings, 
    %   represents the seconds you should be at each respective waypoint
    %
    %************ LOOKUP WAYPOINTS ************************

    % Write code here

    %Sample waypoints for hover trajectory
    if question == 2
        waypoints = [0   0.1 0.2 0.3; 
                     0   0   0   0; 
                     0.5 0.5 0.5 0.5; 
                     0   0   0   0];
        waypoint_times = [0 2 4 6];
    end

    if question == 3
        waypoints = [0 0 0 0 0;
                     0 0 0 0 0;
                     0 1 1 1 0;
                     0 0 0 0 0];
         waypoint_times = [0 2.5 5 7.5 10];
    end

    if question == 4
        % the z position rise from 0 to 1
        % add gravity
        hover_z = 1;
        trajectory_x = [1, 2, 3];
        trajectory_y = [1, 2, 3];

        %0-1 second
        waypoints_takeoff = [0 0; %x
                             0 0; %y
                             0 hover_z; %z
                             0 0]; %yaw
        %2 second
        waypoints_hover = [0; %x
                           0; %y
                           hover_z; %z
                           0;]; %yaw

        %3-6 second
        waypoints_trajectory = [0       trajectory_x(1) trajectory_x(2), trajectory_x(3); %x
                                0       trajectory_y(1) trajectory_y(2), trajectory_y(3); %y
                                hover_z hover_z         hover_z          hover_z; %z
                                0       0               0                0]; %yaw
        %7-9 second                    
        waypoints_land = [trajectory_x(3) trajectory_x(3) trajectory_x(3) trajectory_x(3); %x
                          trajectory_y(3) trajectory_y(3) trajectory_x(3) trajectory_x(3); %y
                          hover_z         0               0               0;
                          0               0               0               0]; %z
        % concatenate the waypoints              
        waypoints = [waypoints_takeoff, waypoints_hover, waypoints_trajectory, waypoints_land];
        waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
    end

    if question == 5
        waypoints = [0 0  0  0;
                     0 0  0  0;
                     0 .1 .1 .1;
                     0 0  0  0];
         waypoint_times = [0 3.3 6.6 10];
    end

    if question == 5.5
        yaw = deg2rad(15);
        waypoints = [0    0    0    0;
                     0    0    0    0;
                     0    .1   .1   .1;
                     0    yaw  yaw  yaw];
        waypoint_times = [0 3.3 6.6 10];
    end

    if question == 7.1 || question == 7.2 || question == 7.3 || question == 7.4 || question == 7.5 || question == 7.6 || question == 7.7
        waypoints = [0   0   0   0   0   0   0   0   0   0   0   0   0;
                     0   0   0   0   0   0   0   0   0   0   0   0   0;
                     0   .5  1  1    1   1   1   1   1   1  1   1   1;
                     0   0   0   0   0   0   0   0   0   0   0   0   0];
        waypoints = waypoints.*10;
        waypoint_times = [0 1 2 3 4 5 6 7 8 9 10 11 12];
    end
    
    if question == 7.7
        waypoints = [0   0   0   0   0   0   0   0   0   0   0   0   0;
                     0   0   0   0   0   0   0   0   0   0   0   0   0;
                     0   1  1   1   1   1   1   1   1   1   1   1   1;
                     0   0   0   0   0   0   0   0   0   0   0   0   0];
        waypoint_times = [0 1 2 3 4 5 6 7 8 9 10 11 12];
        waypoints = waypoints.*10;
    end
    
    if question == 8
        % generate the spline
        x = pi * [0:.5:2]; 
        y = [0  2  0 -2  0  2  0; 
             1  1  2  1  0  1  1];
        pp = spline(x,y);
        yy = ppval(pp, linspace(0,2*pi,2001));
        
        waypoints = [yy;
                     ones(1,2001);
                     ones(1,2001);];
        
        waypoint_times = 0:0.005:10;
    end
    
    if question == 9
        % generate the spline
        x = pi * [0:.5:2]; 
        y = [0  2  0 -2  0  2  0; 
             1  1  2  1  0  1  1];
        pp = spline(x,y);
        yy = ppval(pp, linspace(0,2*pi,2001));
        
        waypoints = [yy;
                     ones(1,2001);
                     ones(1,2001);];
        
        waypoint_times = 0:0.005:10;
    end
end