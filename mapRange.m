function [output] = mapRange(input,outputLow,outputHigh)

input = min(max(input,0),1);
output = outputLow + (input*(outputHigh-outputLow));
end

