classdef movingTarget < target
  %MOVINGTarget Summary of this class goes here
  %   Detailed explanation goes here

  % Copyright 2008 The MathWorks, Inc.

  properties
    deltaAoA
  end

  methods
    function newMovingTarget = movingTarget(AoA, range, signal, deltaAoA)
      % assign the superclass portion
      newMovingTarget = newMovingTarget@target(AoA, range, signal) ;

      if nargin == 4
        % assign the movingTarget's unique property
        newMovingTarget.deltaAoA = deltaAoA ;
      end
    end

    function move(amovingTarget)
      % add change in AoA
      amovingTarget.AoA =  amovingTarget.AoA ...
        + amovingTarget.deltaAoA ;

      % Keep target in in bounds by flipping direction after having
      % moved too far
      if abs(amovingTarget.AoA) > 65
        amovingTarget.deltaAoA = -amovingTarget.deltaAoA ;
      end

    end

  end
end
