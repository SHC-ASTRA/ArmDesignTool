function [forces,moments] = calcActuatorForces(arm)
linkCount = 0;
for axis = arm
    if(axis.length ~= 0)
        linkCount = linkCount + 1;
    end
end

for i = 1:linkCount
    [forces(i),moments(i)] = calcActuatorForce(arm,i);
end

end
