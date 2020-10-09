%% Analyze Sensor Design
% We explore a sensor design for a far-field staring array.
% The array is designed to detect the angle of arrival (AoA)
% for a distant object emitting a signal of some frequency.
%
% The system is tested with multiple targets.  Different
% sensor configurations can be explored to investigate the
% accuracy of AoA detection, as well as the ability of the
% sensor to resolve multiple targets.
%
% Copyright 2008 The MathWorks, Inc.

%% Clear Workspace
clear all
close all

%% Define Radio Beacon
signal.spdOfLight  =    3e8 ;
signal.freq        =   12e7 ;
signal.wavelength  = signal.spdOfLight/signal.freq ;
signal.amp         =      1 ;

%% Define balloon
balloon.AoA    =    -10    ;
balloon.range  =    1e8    ;
balloon.signal = signal    ;

%% Multiple balloons
balloon(2).AoA    =     20 ;
balloon(2).range  =    1e8 ;
balloon(2).signal = signal ;

balloon(3).AoA    =     25 ;
balloon(3).range  =    1e8 ;
balloon(3).signal = signal ;

%% Define Sensor Array
sensor.numDetector = 96       ;
sensor.numSamples  = 1        ;
sensor.sampleRate  = 1/.3e-7  ;
sensor.noiseRatio  = 50       ;
sensor.wavelength  = 3e8/12e7 ;
sensor.spacing     = sensor.wavelength/2;
sensor.sensitivity = 0.8      ;
sensor.minZeroPad  = 2048     ;

%% Create Sensor Field Data from balloon signals  ;
arrayData          = sensorSignal(balloon, sensor) ;

%% Compute Angles of Arrival
arrivalAngles      = computeAoA(sensor,arrayData) ;
disp(arrivalAngles)
