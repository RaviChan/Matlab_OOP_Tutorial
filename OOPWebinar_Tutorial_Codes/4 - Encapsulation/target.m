classdef target
  %TARGET Class for representing a target
  %   A target class knows its angle, signal and range

  % Copyright 2008 The MathWorks, Inc.

  properties
    AoA
    range
    signal
  end

  methods
    function thisTarget = target(AoA, range, signal)
      if nargin == 3
        thisTarget.AoA    = AoA;
        thisTarget.range  = range;
        thisTarget.signal = signal;
      end
    end

    function identify(thisTarget)
      disp(['I am a friend, please don''t shoot!' ...
        'I am arriving at ' num2str(thisTarget.AoA)]);
    end
  end
end
