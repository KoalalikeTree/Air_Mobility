function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
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
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here

g = params.gravity;
psi_des = desired_state.rot(3);

% desired_rot
A = [sin(psi_des) -cos(psi_des);
     cos(psi_des) sin(psi_des)];

x_dot_dot_des = desired_state.acc(1);
y_dot_dot_des = desired_state.acc(2);

B = [x_dot_dot_des; y_dot_dot_des];
phi_theta_des = (1/g) * A * B;
rot = [phi_theta_des(1); phi_theta_des(2); desired_state.rot(3)];

% desired_rot_dot
psi_dot_des = desired_state.omega(3);

C = [cos(psi_des) sin(psi_des);
     -sin(psi_des) cos(psi_des)];

D = [x_dot_dot_des * psi_dot_des;
     y_dot_dot_des * psi_dot_des];

phi_theta_dot_des = (1/g) * C * D;

omega = [phi_theta_dot_des(1); phi_theta_dot_des(2); desired_state.omega(3)];


end

