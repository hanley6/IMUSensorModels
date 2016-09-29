function [T,sigma] = allan(meas,fs,pts)
%ALLAN Computes the Allan variance of a sample of measurements. The
%following document was used to create this code: 
% Freescale Semiconductor. "Allan Variance: Noise Analysis for Gyroscopes."
% Document Number: AN5087. Rev. 0, 2/2015.
% http://www.nxp.com/files/sensors/doc/app_note/AN5087.pdf?fasp=1&WT_TYPE=Application%20Notes&WT_VENDOR=FREESCALE&WT_FILE_FORMAT=pdf&WT_ASSET=Documentation&fileExt=.pdf
%   Inputs:
%           meas = set of measurements (number of rows is number of
%                  measurements)
%           fs = frequency of measurements
%           pts = number of points to plot in Allan variance
%   Outputs:
%           T = Time spans of Allan variance points
%           sigma = Allan standard deviation

% Figure out how big the output data set is
[N,M] = size(meas);

% Determine largest bin size
n = 2.^(0:floor(log2(N/2)))';
maxN = n(end);
endLogInc = log10(maxN);

% Create log spaced vector average factor
m = unique(ceil(logspace(0,endLogInc,pts)))'; 

% t0 = sample interval
t0 = 1/fs; 

% T = length of time for each cluster
T = m*t0;

% Integration of samples over time to obtain output angle ?
theta = cumsum(meas)/fs; 

% Array of dimensions (cluster periods) X (#variables)
sigma2 = zeros(length(T),M);

% Loop over the various cluster sizes
for i=1:length(m) 
    % Implements the summation in the AV equation
    for k=1:N-2*m(i) 
        sigma2(i,:) = sigma2(i,:) + (theta(k+2*m(i),:) - 2*theta(k+m(i),:) + theta(k,:)).^2;
    end
end

sigma2 = sigma2./repmat((2*T.^2.*(N-2*m)),1,M);
sigma = sqrt(sigma2);
end