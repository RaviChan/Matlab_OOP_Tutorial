classdef target
  %TARGET Definition of all Targets
  %   This is a target class for sensor modelling

  % Copyright 2008 The MathWorks, Inc.

  properties
    AoA
    range
    signal
  end

  methods
    function newTarget = target(AoA, range, signal)
      if nargin == 3
        newTarget.AoA    = AoA    ;
        newTarget.range  = range  ;
        newTarget.signal = signal ;
      end
    end

    function identify(thisTarget)
      disp(['I am a friend, please don''t shoot. ', ...
        'I am arriving at ' num2str(thisTarget.range)]) ;
    end
  end
end
