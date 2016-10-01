% This script processes data and performs analysis of measured signal in
% terms of power spectral density and Allan variance
%   Written By: David Hanley (9/28/2016)
clear all;
clc;
close all;

%-------------------------------------------------------------------------%
%---------------------- Data Loading and Analysis ------------------------%
%-------------------------------------------------------------------------%
% Load Data
load('Data002.mat')

% Parse data
meas = input('Input measurements as a column vector: ');
time = input('Input timestamps associated with measurements: ');
pts = input('Input number of points to plot in Allan variance result: ');

% Compute sampling frequency
fs = 1/median(diff(time));

% Compute Allan Variance From Data
[T,sigma] = allan(meas,fs,pts);

% Compute PSD from Data
[Pxx,f] = computePowerSpectralDensities( meas,fs );

%----------------------------- Plot Results ------------------------------%
% Allan Variance
figure(1)
subplot(2,1,1)
loglog(T,sigma.^2)
xlabel('Window Time (s)')
ylabel('Allan Variance')
title('Allan Variance Plot')
grid on;

% Power Spectral Densities
subplot(2,1,2)
loglog(f,Pxx)
xlabel('Frequency (Hz)')
ylabel('Power')
title('PSD Plot')
grid on;

% Allan Deviation
figure(2)
loglog(T,sigma)
xlabel('Window Time (s)')
ylabel('Allan Deviation')
title('Allan Deviation Plot')
grid on;
%--------------------------- End Plot Results ----------------------------%

%-------------------------------------------------------------------------%
%-------------------- End Data Loading and Analysis ----------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%----------------- Compute Critical System Parameters --------------------%
%-------------------------------------------------------------------------%

% Find Allan Deviation Along Angle Random Walk Portion of Curve
tau_low = input('Input left side lower time window bound: ');
tau_high = input('Input left side higher time window bound: ');
count = 1;
for i = 1:length(T)
    if (T(i) >= tau_low && T(i) <= tau_high)
        tau(count) = T(i); %#ok<SAGROW>
        ADev(count) = sigma(i); %#ok<SAGROW>
        count = count + 1;
    end
end

% Use bisection method to compute N (the mean square value of the angle
% random walk noise)
higher = 1;
lower = 1e-10;
for i = 1:100
    N = (higher+lower)/2;
    error = sum(N./sqrt(tau) - ADev);
    if error < 0 
        lower = N;
    else 
        higher = N;
    end
end

% Find Allan Deviation Along Rate Random Walk Portion of Curve
tau_low = input('Input right side lower time window bound: ');
tau_high = input('Input right side higher time window bound: ');
count = 1;
for i = 1:length(T)
    if (T(i) >= tau_low && T(i) <= tau_high)
        tau_right(count) = T(i); %#ok<SAGROW>
        ADev_right(count) = sigma(i); %#ok<SAGROW>
        count = count + 1;
    end
end

% Use bisection method to compute K (the mean square value of the rate
% random walk noise)
higher = 1;
lower = 1e-10;
for i = 1:100
    K = (higher+lower)/2;
    error = sum(K*sqrt(tau_right/3) - ADev_right);
    if error < 0
        lower = K;
    else
        higher = K;
    end
end

% Find the Time Window at Minimum of Allan Variance
index = find(sigma == min(sigma));
Tc = T(index);

% Find B (the mean square value of the bias instability)
B = min(sigma)/sqrt(2*log(2)/pi);

% Find the standard deviation of total sensor measurements (assuming error
% mean is zero)
sig_gyro_noise = sqrt(var(meas));

% Choose standard deviation used in rate random walk contribution so that 
% low frequency of RRW PSD matches low frequency of the PSD from dataset of
% sensor data
sig_w = 10;

% Display Parameter Results
display(sprintf('B: %f',B));
display(sprintf('N: %f',N));
display(sprintf('K: %f',K));
display(sprintf('sig_gyro_noise: %f',sig_gyro_noise));
display(sprintf('T: %f',Tc));

% Show resulting parameter lines on Allan deviation plot
figure(3)
loglog(T,sigma,T,N./sqrt(T),'--',T,B*sqrt(2*log(2)/pi)*ones(1,length(T)),'--',T,K*sqrt(T/3),'--')
xlabel('Window Time (s)')
ylabel('Allan Deviation')
title('Allan Deviation Plot')
grid on;
%-------------------------------------------------------------------------%
%--------------- End Compute Critical System Parameters ------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%--------------------------- Test Simulation -----------------------------%
%-------------------------------------------------------------------------%
% Initialize Bias, Rate Random Walk, Simulation Measurement Vector, and 
% Rate Random Walk Vector 
bias = 0;
rrw = 0;
S = 0;
Bf = 0;
sim_meas = zeros(length(meas),1);
rrw_save = zeros(length(meas),1);

% Run Simulation Over Same Time and at Same Data Rate as Test Data
for i = 1:length(meas)
    sensor_meas = SensorDetModel( 0,S,Bf );
    [sim_meas(i,1),bias,rrw] = SensorStocModel( sensor_meas,B,K,Tc,fs,bias,sig_gyro_noise,rrw,sig_w );
    rrw_save(i,1) = rrw;
end

% Compute Allan Variance of Simulation Data
[T_val,sigma_val] = allan(sim_meas,fs,100);

% Compute PSD of Simulation Data
[Pxx_val,f_val] = computePowerSpectralDensities( sim_meas,fs );

% Compute PSD of Rate Random Walk Data
[Pxx_rrw,f_rrw] = computePowerSpectralDensities( rrw_save,fs );

% Compare Data Allan Deviation and Simulation Allan Deviation
figure(4)
loglog(T_val,sigma_val,T,sigma)
xlabel('Window Time (s)')
ylabel('Allan Deviation')
title('Allan Deviation Comparison')
legend('Simulation','Data')
grid on;

% Compare Data Power Spectral Density and Simulation Power Spectral Density
figure(5)
loglog(f,Pxx,f_val,Pxx_val,'r')
xlabel('Frequency (Hz)')
ylabel('Power')
title('PSD Plot Comparison')
legend('Data','Simulation')
grid on;

% Compare Data Measurements with Simulation Measurements
figure(6)
plot(time,meas,time,sim_meas,'r')
xlabel('Time (sec)')
ylabel('Signal Measurement')
grid on;
legend('Data','Simulation')
title('Signal Output Comparison')

% Compare Data PSD with Rate Random Walk Power Spectral Density
figure(7)
loglog(f,Pxx,f_rrw,Pxx_rrw,'r')
xlabel('Frequency (Hz)')
ylabel('Power')
title('PSD Plot RRW and Data')
legend('Data','RRW')
grid on;
%-------------------------------------------------------------------------%
%------------------------- End Test Simulation ---------------------------%
%-------------------------------------------------------------------------%