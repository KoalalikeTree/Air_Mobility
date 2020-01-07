function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%   [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; rpm1; rpm2; rpm3, rpm4];
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
%   Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************
m = params.mass;
g = params.gravity;
I  = params.inertia;

phi = state(7);
theta = state(8);
psi = state(9);

% eul = [phi, theta, psi];
% bRw = eul2rotm(eul,'XYZ');
% wRb = bRw';

% % Nathan-simplified
% agu_vel = state(10:12);
Reb = [
    cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta),-cos(phi)*sin(psi),cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
    cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),cos(phi)*cos(psi),sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);
    -cos(phi)*sin(theta),sin(phi),cos(phi)*cos(theta)
    ];
% A = [m * eye(3), zeros(3);
%         zeros(3), I];
% B = Reb * [0; 0; F] - [0; 0; m * g];
% C = [B; (cross(agu_vel, I * agu_vel))];
% line_ang_vel = inv(A) \ C;

% linear acceleration
accel = 1 / m * (Reb * [0; 0; F] - [0; 0; m * g]);

% Angular velocity
agu_vel = [state(10); state(11); state(12)];
agu_accel = inv(I) * (M - cross((agu_vel), I * agu_vel));

% State_dot
state_dot = zeros(16,1);
state_dot(1:3) = state(4:6);
state_dot(4) = g * (theta * cos(psi) + phi * sin(psi));
state_dot(5) = g * (theta * sin(psi) - phi * cos(psi));
state_dot(6) = F/m - g;
% state_dot(4:6) = accel(1:3);
% state_dot(4:6) = line_ang_vel(1:3);
state_dot(7:9) = state(10:12);
% state_dot(10:12) = agu_accel(1:3);
% state_dot(10:12) = line_ang_vel(4:6);
state_dot(10) = M(1)/I(1,1);
state_dot(11) = M(2)/I(2,2);
state_dot(12) = M(3)/I(3,3);
state_dot(13:16) = rpm_motor_dot(1:4);
    
end