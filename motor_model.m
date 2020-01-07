function [F_motor,M_motor,rpm_motor_dot] = motor_model(F,M,motor_rpm,params)

% Input parameters
% 
%   F,M: required force and moment
% 
%   motor_rpm: current motor RPM
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   F_motor: Actual thrust generated by Quadcopter's Motors
%
%   M_motor: Actual Moment generated by the Quadcopter's Motors
%
%   rpm_dot: Derivative of the RPM
%
%************ MOTOR MODEL ************************

% Write code here

d = params.arm_length;
CT = params.thrust_coefficient;
CQ = params.moment_scale;

rpm2FM = [CT      CT     CT    CT;
          0       d*CT   0     -d*CT;
          -d*CT   0      d*CT  0;
          -CQ     CQ     -CQ   CQ;];
      
FM = rpm2FM * motor_rpm.^2;
% w_des = inv(rpm2FM) * FM;
F_motor = FM(1);
M_motor = FM(2:4);
w_des = sqrt(rpm2FM \ [F;M]);

w_des(w_des>20000) = 20000;
% w_des(w_des>40000) = 40000;
w_des(w_des<3000) = 3000;

rpm_motor_dot = params.motor_constant * (w_des - motor_rpm);
rpm_motor_dot = 46.5 * (w_des - motor_rpm);

end
