function q_dot = Quaternion_derivative(q, w)
% Calculates the quaternion derivative using derived solution for Omega matrix
wx = w(1);
wy = w(2);
wz = w(3);
Omega = [0 , wz , -wy, wx; 
        -wz, 0  , wx , wy; 
         wy, -wx, 0  , wz; 
        -wx, -wy, -wz, 0];
q_dot = 0.5 * (Omega @ q);
end