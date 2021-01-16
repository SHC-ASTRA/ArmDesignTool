function [stop ] = bestArmPlotFcn(optimValues,state)

if strcmp(state,'iter') && optimValues.iteration>1
    cla
    x = optimValues.bestx;
    
    [arm, angleMins, angleMaxs] = x2Arm(x);
    
    [arm1,~] = forwardKinematics(arm, (angleMins+angleMaxs)./2 );
    drawArm(arm1)
    
end

stop = false;

end

