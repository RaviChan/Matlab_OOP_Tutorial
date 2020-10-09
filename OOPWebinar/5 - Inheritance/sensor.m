classdef sensor
  %SENSOR Class for Sensor Staring Array
  %   This is the prototype class for the sensor Staring Array.

  % Copyright 2008 The MathWorks, Inc.

  properties
    numDetector = 96
  end

  properties (Access = protected)
    numSamples  = 1
    sampleRate  = 1/.3e-7
    noiseRatio  = 50
    frequency   = 12e7
    sensitivity = 0.8
    minZeroPad  = 2048
  end

  properties (Constant)
    spdLight = 3e8
  end

  properties (Dependent)
    wavelength
    spacing
  end

  methods
    %% wavelength getter function
    function wavelength = get.wavelength(theSensor)
      wavelength = sensor.spdLight / theSensor.frequency;
    end

    %% spacing getter function
    function spacing = get.spacing(theSensor)
      spacing = theSensor.wavelength / 2;
    end

    %% Compute AoA method
    function arrivalAngles = computeAoA(  theSensor, target   )
      arrayData          = sensorSignal(theSensor, target   ) ;
      arrivalAngles      = determineAoA(theSensor, arrayData) ;
    end
  end

  methods (Access = protected)
    %% Sensor Signal method for synthesizing sensor data
    function arrayData = sensorSignal(theSensor, target)
      arrayData = computeSensorSignal(theSensor, target) ;
    end
  end
end

%% Helper Sub-Functions for Class Methods
function arrayData = computeSensorSignal(sensor, target)

% Initialize output vector
arrayData  = zeros(sensor.numSamples,sensor.numDetector);

% Identify total number of targets
numTargets = length(target)                             ;

% Detector Positions (m)
orgX       = (sensor.numDetector+1)/2;
detectPos  = ((1:sensor.numDetector) - orgX)*sensor.spacing;%Sensors row vector

% For each source/target, calculate the amplitue at the array and sum
for n=1:numTargets
  bearing   = 0.5*pi-target{n}.AoA*pi/180          ; %Source angle
  tx        = target{n}.range*cos(bearing)         ; %Source X position
  ty        = target{n}.range*sin(bearing)         ; %Source Y position
  distance  = sqrt((tx+detectPos).^2+(ty)^2)       ; %Distance to source
  timeDelay = distance/target{n}.signal.spdOfLight ; %Time delay to source
  for k=1:sensor.numSamples
    arrayData(k,:) = arrayData(k,:) + ...
      target{n}.signal.amp*exp(1j*(timeDelay-k/sensor.sampleRate)* ...
      2*pi*target{n}.signal.freq);
  end
end

% Add noise
for k=1:sensor.numSamples
  arrayData(k,:)=noisyvar(arrayData(k,:).',sensor.noiseRatio,numTargets).';
end

end

function y=noisyvar(x,snr,num)
% Add noise to given signal
A     = 1;
noise = (A/(sqrt(2)))*randn(length(x),1)+(A/(sqrt(2)))*1j*randn(length(x),1);
powx  = (x'*x)/length(x)/num;
pown  = (noise'*noise)/length(x);
rwant = 10^(snr/10);
nwant = (noise/sqrt(pown))*sqrt(powx/rwant);
y     = x+nwant;
end


function targetAoA = determineAoA(sensor,arrayData)
% COMPUTEAOA  Estimate the direction of arrival of the sources in the
% sensor array data set using simplistic peak finding method
% Example:
%  angels=doa(ds)

%% Determine zero pad size
nearestPower2  = 2^ceil(log2(sensor.numDetector))         ;
zeroPadTo      = max(nearestPower2, sensor.minZeroPad)    ;

%% Compute the FFT of the sensor data
[mags, angles] = magfft(sensor,arrayData,zeroPadTo)       ;

%% Compute the amplitudes
detectRange    = sensor.sensitivity*max(mags)             ;%dynamic detect range
targetIndex    = peakdet(mags,detectRange)                ;%find the peaks

if ~isempty(targetIndex)
  targetAoA   = sort(angles(targetIndex(:,1))*180)       ;%angles
else
  targetAoA   = []                                       ;%angles
end

%% Plot the results
semilogy(angles*180,mags(1:zeroPadTo),'b')                ;
title ([num2str(length(targetAoA)) ' Identified Targets']);
xlabel('Degrees')                                     ;
ylabel('Amplitude'  )                                     ;

%% Plot vertical lines on the targets
for n = 1:length(targetAoA)
  vline(targetAoA, 'r')                                  ;
end

end % function computeAoA

function [maxtab, mintab]=peakdet(v, delta)
%PEAKDET Detect peaks in a vector
%        [MAXTAB, MINTAB] = PEAKDET(V, DELTA) finds the local
%        maxima and minima ("peaks") in the vector V.
%        A point is considered a maximum peak if it has the maximal
%        value, and was preceded (to the left) by a value lower by
%        DELTA. MAXTAB and MINTAB consists of two columns. Column 1
%        contains indices in V, and column 2 the found values.
% Eli Billauer, 3.4.05 (Explicitly not copyrighted).
% http://www.billauer.co.il/peakdet.html
% This function is released to the public domain; Any use is allowed.

maxtab = [];
mintab = [];

v = v(:); % Just in case this wasn't a proper vector

if (length(delta(:)))>1
  error('Input argument DELTA must be a scalar');
end

if delta <= 0
  error('Input argument DELTA must be positive');
end

mn = Inf; mx = -Inf;
mnpos = NaN; mxpos = NaN;

lookformax = 1;

for i=1:length(v)
  this = v(i);
  if this > mx, mx = this; mxpos = i; end
  if this < mn, mn = this; mnpos = i; end

  if lookformax
    if this < mx-delta
      maxtab = [maxtab ; mxpos mx]; %#ok
      mn = this; mnpos = i;
      lookformax = 0;
    end
  else
    if this > mn+delta
      mintab = [mintab ; mnpos mn]; %#ok
      mx = this; mxpos = i;
      lookformax = 1;
    end
  end
end

end % function peakdet

function [mags, freq]=magfft(sensor,arrayData,zeroPadTo)
% MAGFFT Calculate the magnitude square of the FFT of the
% sensor array sample data, zeropadding by zeroPadTo elements
% Example:
%  result=magfft(s,128);

%% Preallocate store of magnitudes and frequencies
mag   = zeros(sensor.numSamples,zeroPadTo)  ;
freq  = asin((-0.5:1/(zeroPadTo-1):0.5)*sensor.wavelength/sensor.spacing)/pi;

%% Take the sum over each sensor array sample
for k=1:sensor.numSamples
  avbig                       = zeros(1,zeroPadTo)        ;%Zero pad
  avbig(1:sensor.numDetector) = arrayData(1,:)            ;
  response                    = fft(avbig)/zeroPadTo      ;%normalized fft
  mag(k,:)                    = abs(fftshift(response)).^2;%Mag squared
end

%% Combine the results if available
if sensor.numSamples > 1
  mags = sum(mag);
else
  mags = mag;
end

end % funciton magfft

