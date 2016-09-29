% Interpret IMU Direction Test Data
% Initialization
clear all;
clc;
close all;

% Import Data
filename = input('Input file name: ','s');
A = importdata(filename);
Time = A.data(:,1);
acc_x = A.data(:,2);
acc_y = A.data(:,3);
acc_z = A.data(:,4);
gyro_roll = A.data(:,5);    % About body-x
gyro_pitch = A.data(:,6);   % About body-y
gyro_yaw = A.data(:,7);     % About body-z

% Test Data Rate
data_rate = 1./diff(Time);
display(sprintf('Mean Data Rate: %f Hz',mean(data_rate)));
display(sprintf('Variance of Data Rate: %f Hz',var(data_rate)));

% Convert accel data
accel = [acc_x, acc_y, acc_z]'.*9.80665./10000;
accel_norm = sqrt(sum(accel.^2,1));

% Convert gyro data
gyro = [gyro_roll, gyro_pitch, gyro_yaw]'.*0.0154.*pi./180;
index = 1;
for i = 1:length(gyro(1,:))
    if (abs(gyro(1,i)) < 15 && abs(gyro(2,i)) < 15 && abs(gyro(3,i)) < 15)
        gyro_rev(:,index) = gyro(:,i);
        time_rev(index) = Time(i);
        index = index + 1;
    end
end

% Save Results
save('Data','accel','gyro_rev','Time','time_rev');

% Produce plots
figure(1)
plot(Time-Time(1),accel(1,:),Time-Time(1),accel(2,:),Time-Time(1),accel(3,:),Time-Time(1),accel_norm)
xlabel('Time (s)')
ylabel('Specific Force (m/s^2)')
legend('a_x','a_y','a_z','||a||')
grid on;

figure(2)
plot(Time-Time(1),gyro(1,:),Time-Time(1),gyro(2,:),Time-Time(1),gyro(3,:))
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend('\omega_x','\omega_y','\omega_z')
grid on;

figure(3)
plot(time_rev-time_rev(1),gyro_rev(1,:),time_rev-time_rev(1),gyro_rev(2,:),time_rev-time_rev(1),gyro_rev(3,:))
xlabel('Time (s)')
ylabel('Angular Velocity Corrected (rad/s)')
legend('\omega_x','\omega_y','\omega_z')
grid on;