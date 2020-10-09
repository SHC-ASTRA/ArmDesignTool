function [moment] = calcMoment(arm,axisNum)
g = 9.80665;
moment = 0;
baseX = arm(axisNum).B_x;


for i = axisNum:length(arm)
    moment = moment + g*arm(i).originMass*(arm(i).B_x - baseX);
    if arm(i).length > 0
        moment = moment + g*arm(i).lengthMass*((arm(i).B_x+arm(i).T_x)/2 - baseX);
    end
end

