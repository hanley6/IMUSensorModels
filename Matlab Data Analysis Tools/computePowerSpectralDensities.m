function [ Pxx,f ] = computePowerSpectralDensities( meas,fs )
%COMPUTEPOWERSPECTRALDENSITIES Computes the one-sided power spectral
%density of set of measurements
%   Inputs:
%           meas = set of measurements
%           fs = sampling frequency of measurements
%   Outputs:
%           Pxx = power spectra of measurements

[Pxx,f] = pwelch(meas,[],[],[],fs,'onesided');

end

