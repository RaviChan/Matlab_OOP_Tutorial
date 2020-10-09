function arrayData = sensorSignal(target, sensor)

% Copyright 2008 The MathWorks, Inc.

% Initialize output vector
arrayData  = zeros(sensor.numSamples,sensor.numDetector);

% Identify total number of targets
numTargets = length(target)                             ;

% Detector Positions (m)
orgX       = (sensor.numDetector+1)/2;
detectPos  = ((1:sensor.numDetector) - orgX)*sensor.spacing;%Sensors row vector

% For each source/target, calculate the amplitue at the array and sum
for n=1:numTargets
  bearing   = 0.5*pi-target(n).AoA*pi/180             ;%Source angle
  tx        = target(n).range*cos(bearing)            ;%Source X position
  ty        = target(n).range*sin(bearing)            ;%Source Y position
  distance  = sqrt((tx+detectPos).^2+(ty)^2)          ;%Distance to source
  timeDelay = distance/target(n).signal.spdOfLight    ;%Time delay to source
  for k=1:sensor.numSamples
    arrayData(k,:) = arrayData(k,:) + ...
      target(n).signal.amp*exp(j*(timeDelay-k/sensor.sampleRate)* ...
      2*pi*target(n).signal.freq);
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
noise = (A/(sqrt(2)))*randn(length(x),1)+(A/(sqrt(2)))*j*randn(length(x),1);
powx  = (x'*x)/length(x)/num;
pown  = (noise'*noise)/length(x);
rwant = 10^(snr/10);
nwant = (noise/sqrt(pown))*sqrt(powx/rwant);
y     = x+nwant;
end
