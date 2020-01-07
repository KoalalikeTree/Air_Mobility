function [] = Q7()
    des_positionMatrix = zeros(7, 2001);
    act_positionMatrix = zeros(7, 2001);
    des_accelerationMatrix = zeros(7, 2001);
    act_accelerationMatrix = zeros(7, 2001);
    for i = 1:7
        params = struct(...
        'mass',                   0.770, ...
        'gravity',                9.80665, ...
        'arm_length',           0.1103, ...
        'motor_spread_angle',     0.925, ...
        'thrust_coefficient',     8.07e-9, ...
        'moment_scale',           1.3719e-10, ...
        'motor_constant',        36.5, ...%36.5
        'rpm_min',             3000, ...
        'rpm_max',            40000, ...
        'inertia',            diag([0.0033 0.0033 0.005]),...
        'COM_vertical_offset',                0.05);
    
        [waypoints, waypoint_times] = lookup_waypoints( 7 + i*0.1);

        
        time_initial = 0; 
        time_final = 10;
        time_step = 0.005; % sec

        time_vec = time_initial:time_step:time_final;
        max_iter = length(time_vec);
        
        state = zeros(16,1);
        
        state(1)  = waypoints(1,1); %x
        state(2)  = waypoints(2,1); %y
        state(3)  = waypoints(3,1); %z
        state(4)  =  0;        %xdot
        state(5)  =  0;        %ydot
        state(6)  =  0;        %zdot
        state(7) =   0;         %phi
        state(8) =   0;         %theta
        state(9) =   waypoints(4,1); %psi
        state(10) =  0;         %phidot 
        state(11) =  0;         %thetadot
        state(12) =  0;         %psidot
        state(13:16) =  0;      %rpm
        
        trajectory_matrix = trajectory_planner( 7 + i*0.1, waypoints, max_iter, waypoint_times, time_step);
        
        actual_state_matrix = zeros(15, max_iter);  
        % [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

        actual_state_matrix(:,1) = vertcat(state(1:12), 0, 0, 0);

        actual_desired_state_matrix = zeros(15, max_iter);      

        for iter = 1:max_iter-1

            % convert current state to stuct for control functions
            current_state.pos = state(1:3);
            current_state.vel = state(4:6);
            current_state.rot = state(7:9);
            current_state.omega = state(10:12);
            current_state.rpm = state(13:16);

            % Get desired state from matrix, put into struct for control functions
            desired_state.pos = trajectory_matrix(1:3,iter);
            desired_state.vel = trajectory_matrix(4:6,iter);
            desired_state.rot = trajectory_matrix(7:9,iter);
            desired_state.omega = trajectory_matrix(10:12,iter);
            desired_state.acc = trajectory_matrix(13:15,iter);

            % Get desired acceleration from position controller
            [F, desired_state.acc] = position_controller(current_state, desired_state, params, 7);

            % Computes desired pitch and roll angles
            [desired_state.rot, desired_state.omega] = attitude_planner(desired_state, params);

            % Get torques from attitude controller
            M = attitude_controller(current_state, desired_state, params, 7);

            % Motor model
            [F_actual, M_actual, rpm_motor_dot] = motor_model(F, M, current_state.rpm, params);

            % Get the change in state from the quadrotor dynamics
            timeint = time_vec(iter:iter+1);
            [tsave, xsave] = ode45(@(t,s) dynamics(params, s, F_actual, M_actual, rpm_motor_dot), timeint, state);
            state    = xsave(end, :)';
            acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));

            % Update desired state matrix
            actual_desired_state_matrix(1:3,iter+1) =  desired_state.pos;
            actual_desired_state_matrix(4:6, iter+1) = desired_state.vel;
            actual_desired_state_matrix(7:9, iter+1) = desired_state.rot;
            actual_desired_state_matrix(10:12, iter+1) = desired_state.omega;
            actual_desired_state_matrix(13:15, iter+1) = desired_state.acc;

            % Update actual state matrix
            actual_state_matrix(1:12, iter+1) = state(1:12);
            actual_state_matrix(13:15, iter+1) = acc;
        end
        
        plot_quadrotor_errors(actual_state_matrix, actual_desired_state_matrix, time_vec)
        des_positionMatrix(i, :) = actual_desired_state_matrix(3, :);
        act_positionMatrix(i, :) = actual_state_matrix(3, :);
        des_accelerationMatrix(i, :) = actual_desired_state_matrix(15, :);
        act_accelerationMatrix(i, :) = actual_state_matrix(15, :);
        
        
    end
    act_biggestAcc = max(act_accelerationMatrix,[],2);
    des_biggestAcc = max(des_accelerationMatrix,[],2);
    errorMatrix = act_accelerationMatrix - des_accelerationMatrix;
    
    time_initial = 0; 
    time_final = 10;
    time_step = 0.005; % sec
    time_vec = time_initial:time_step:time_final;
    
    
    %%
    subplot(1,2,1);
    for h = 1:7     
        plot(time_vec, act_accelerationMatrix(h,:));
        hold on;
    end
    %title('Km = 46.5 max_rpm=40000');
    title('Km = 36.5 max_rpm=20000');
    xlabel('time');
    ylabel('actual_acceleration');
    ylim([0 40])
    legendCell = {'rise in 10s', 'rise in 5s', 'rise in 2.5s', 'rise in 1.25s', 'rise in 0.75s', 'rise in 0.3s', 'rise in 0.1s'};
    legend(legendCell)
    
    subplot(1,2,2);
    for h = 1:7     
        plot(time_vec, -errorMatrix(h,:));
        hold on;
    end
    xlabel('time');
    ylabel('errors in acceleration');
    ylim([0 220])
    legendCell = {'rise in 10s', 'rise in 5s', 'rise in 2.5s', 'rise in 1.25s', 'rise in 0.75s', 'rise in 0.3s', 'rise in 0.1s'};

    legend(legendCell)
     
end