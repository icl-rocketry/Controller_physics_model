function [measured_accel, measured_gyro] = get_sensor_readings(true_acc, true_omega)
acc_bias = [0.05, -0.02, 0.1];
gyro_bias = [0.001, 0.001, 0.001];

acc_noise = 0 + 0.1 * randn(1, 3);
gyro_noise  = 0 + 0.01 * randn(1, 3); 

measured_accel = true_acc + acc_bias + acc_noise;
measured_gyro = true_omega + gyro_bias + gyro_noise;
end