function [M] = attitude_controller(state,desired_state,params,question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
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
%   M: u2 or moment [M1; M2; M3]
%
%************  ATTITUDE CONTROLLER ************************

% Example PD gains
Kpphi = 190;
Kdphi = 30;

Kptheta = 198;
Kdtheta = 30;

Kppsi = 80;
Kdpsi = 17.88;

if question == 5.5
    Kdpsi = 40;
end

if question == 2
    Kptheta = 600;
    Kdtheta = 70;
end

Kp_ang = [Kpphi; Kptheta; Kppsi];
Kd_ang = [Kdphi; Kdtheta; Kdpsi];

M = params.inertia * (- Kp_ang .* (state.rot - desired_state.rot) - Kd_ang .* (state.omega - desired_state.omega));

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
    x= [state.pos;state.rot;state.vel;state.omega];
    u = - inv(C / (A - B * K) * B) * v - K * x;
    
    M = u(2:4);
else
    Kp_ang = [Kpphi; Kptheta; Kppsi];
    Kd_ang = [Kdphi; Kdtheta; Kdpsi];
    
    rot_des = desired_state.rot;
    rot = state.rot;
    omega_des = desired_state.omega;
    omega = state.omega;
    
    M = I * (Kp_ang.*(rot_des-rot) + Kd_ang.*(omega_des-omega));    
end

end