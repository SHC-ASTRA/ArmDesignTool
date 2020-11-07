function [moment,weight] = calcMoment(arm,axisNum)
g = 9.80665;
moment = 0;
weight = 0;
baseX = arm(axisNum).B_x;


for i = axisNum:length(arm)
    weight = weight + g*arm(i).originMass;
    moment = moment + g*arm(i).originMass*(arm(i).B_x - baseX);
    if arm(i).length > 0
        weight = weight + g*arm(i).lengthMass;
        moment = moment + g*arm(i).lengthMass*((arm(i).B_x+arm(i).T_x)/2 - baseX);
    end
end

