function [moments] = calcMoments(arm)
linkCount = 0;
for axis = arm
    if(axis.length ~= 0)
        linkCount = linkCount + 1;
    end
end

for i = 1:linkCount
    moments(i) = calcMoment(arm,i);
end

end

