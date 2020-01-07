function [F, acc] = position_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], x
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% Example PD gains
Kp1 = 17;
Kd1 = 6.6;

Kp2 = 17;
Kd2 = 6.6;

Kp3 = 20;
Kd3 = 9;
if question == 2 % okay
    % Kp1 = 17;
    Kp1 = 25; % Kd1 = 9
    % Kd1 = 6.6;
    Kd1 = 9; % Kp1= 25
    % Kp2 = 17;
    Kp2 = 17;
    % Kd2 = 6.6;
    Kd2 = 6.6;
    % Kp3 = 20;
    Kp3 = 40;
    Kd3 = 9;
end


if  question == 3.2 % not okay
    Kp1 = 17;
    Kd1 = 6.6;

    Kp2 = 17;
    Kd2 = 6.6;

    Kp3 = 600;
    Kd3 = 180;
end

if question == 4 % this one is okay
    Kp1 = 17;
    Kd1 = 6.6;
    Kp2 = 17;
    Kd2 = 6.6;
    Kp3 = 20;
    Kd3 = 9;
end

if question == 5
     Kp1 = 17;
     Kd1 = 6.6;
     Kp2 = 17;
     Kd2 = 6.6;
     % Kp3 = 480;
     Kp3 = 20;
     % Kd3 = 190;
     Kd3 = 80;
%     Kp1 = 45;
%     Kd1 = 18.6;
%     Kp2 = 30;
%     Kd2 = 18.6;
%     Kp3 = 17;
%     Kd3 = 9;
end

if question == 5.5
     Kp1 = 17;
     Kd1 = 6.6;
     Kp2 = 17;
     Kd2 = 6.6;
     Kp3 = 20;
     Kd3 = 20;
end

if question == 7
    Kp1 = 17;
    Kd1 = 6.6;
    Kp2 = 17;
    Kd2 = 6.6;
    Kp3 = 20;
    Kd3 = 9;
end

if question == 8
    Kp1 = 17;
    Kd1 = 6.6;
    Kp2 = 17;
    Kd2 = 6.6;
    Kp3 = 20;
    Kd3 = 9;
end

if question == 9
    Kp1 = 17;
    Kd1 = 6.6;
    Kp2 = 17;
    Kd2 = 6.6;
    Kp3 = 20;
    Kd3 = 9;
end


% Write code here
Kp = [Kp1;Kp2;Kp3];
Kd = [Kd1;Kd2;Kd3];

acc = desired_state.acc - Kd.*(current_state.vel - desired_state.vel) - Kp.*(current_state.pos - desired_state.pos);

% r¡§1,des = g(¦Èdes cos ¦×T + ¦Õdes sin ¦×T)
% phi_des = desired_state.rot(1); 
% theta_des = desired_state.rot(2);
% psi_des = desired_state.rot(3);
% acc(1) = params.gravity * (theta_des * cos(psi_des) + phi_des * sin(psi_des));
% acc(2) = params.gravity * (theta_des * sin(psi_des) + phi_des * cos(psi_des));
F = params.mass*(params.gravity + acc(3));

g = params.gravity;
I = params.inertia;
m = params.mass;
if question == 6.2 || question == 6.3 || question == 6.5
    A =[
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 g*sin(desired_state.rot(3)) g*cos(desired_state.rot(3)) 0 0 0 0 0 0 0;
     0 0 0 -g*cos(desired_state.rot(3)) g*sin(desired_state.rot(3)) 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 (I(2,2)-I(3,3))* desired_state.omega(3)^2 / I(1,1) 0 0 0 0 0 0 (I(2,2)-I(3,3))* desired_state.omega(3)/I(1,1) 0;
     0 0 0 0 (I(1,1)-I(3,3))* desired_state.omega(3)^2 / I(2,2) 0 0 0 0 -(I(1,1)-I(3,3))* desired_state.omega(3)/I(2,2) 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];
    
    B = [
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 1/I(1,1) 0 0;
     0 0 1/I(2,2) 0;
     0 0 0 1/I(3,3)];
    
    Q = diag([30 30 30 1 1 1 5 5 5 1 1 1]);
    R = diag([10 20 30 10]);
    
    [K, S, P] = lqr(A,B,Q,R);
    
    C = [
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
    ];
    
    v = [desired_state.pos;desired_state.rot(3)];
    x= [current_state.pos;current_state.rot;current_state.vel;current_state.omega];
    u = - inv(C / (A - B * K) * B) * v - K * x;
    F = u(1) + m * g;
    acc = [0;0;0]; 
else
    Kp = [Kp1;Kp2;Kp3];
    Kd = [Kd1;Kd2;Kd3];
    acc = desired_state.acc - Kd.*(current_state.vel - desired_state.vel) - Kp.*(current_state.pos - desired_state.pos);
    F = params.mass*(params.gravity + acc(3));   
end

end
