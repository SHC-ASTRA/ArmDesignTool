function [state,options,optchanged] = bestArmPlotFcn(options,state,flag)

if strcmp(flag,'iter') && length(state.Population)>1
    cla
    
    ibest = state.Best(end);
    ibest = find(state.Score == ibest,1,'last');
    x = state.Population(ibest,:);
    
    [arm, angleMins, angleMaxs] = x2Arm(x);
    
    [arm1,~] = forwardKinematics(arm, (angleMins+angleMaxs)./2 );
    drawArm(arm1)
    
end

optchanged = false;

end

