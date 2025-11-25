function [F_thrust_body, tau_thrust_body, Isp] = Engine_thrust_and_Torque(Fmax, throttle, gimbal_angle_y, gimbal_angle_z, r_CoM, r_engine)
% Obtains the thrust and torque due to the engine from the rockets inertial frame.
F_thrust = Fmax * throttle;
total_gimble = sqrt((gimbal_angle_y ^ 2) + (gimbal_angle_z ^ 2));

Fx = F_thrust * sin(gimbal_angle_y);
Fy = F_thrust * sin(gimbal_angle_z);
Fz = F_thrust * cos(total_gimble);

F_thrust_body = [Fx, Fy, Fz];
tau_thrust_body = cross((r_engine - r_CoM), F_thrust_body);
Isp = 280; % Hard coded for now but make this a function
end