function [score] = evalArm(x)

warning('off','MATLAB:polyshape:repairedBySimplify')

safetyFactor = 1.25;
actuatorMaxForce = 330*4.44822; %Newtons
maxForce = actuatorMaxForce/safetyFactor; %Newtons
actuatorFixedLength = 4.53*0.0254; %Meters
actuatorDesiredStroke = [8*0.0254 8*0.0254 6*0.0254]; %Meters

[arm, angleMins, angleMaxs] = x2Arm(x);

[~,~,force_list,~,actLength_list,~,endPosList,minX,maxX] = simArm(arm,angleMins,angleMaxs,300,maxForce);

score = 0;
strokePenalty = 5000;
overForcePenalty = 500;
stowedSizePenalty = 5;
minStowedSize = 0.5;
workspaceReward = 100;
angleRangeReward = 1;
leverLengthPenalty = 600;

for i = 1:length(angleMins)
    %disp("Axis "+(i+1)+" min extension = "+min(actLength_list(:,i))+" m ( "+(min(actLength_list(:,i))/0.0254)+" in )")
    %disp("Axis "+(i+1)+" max extension = "+max(actLength_list(:,i))+" m ( "+(max(actLength_list(:,i))/0.0254)+" in )")
    
    %fixed+stroke = min
    %stroke = min-fixed
    
    %fixed+2stroke = max
    %2stroke = max-fixed
    %stroke = (max-fixed)/2
    
    maxstroke = min(actLength_list(:,i))-actuatorFixedLength;
    minstroke = (max(actLength_list(:,i))-actuatorFixedLength)/2;
    %idealstroke = (minstroke+maxstroke)/2;
    %disp("Axis "+(i+1)+" stroke length range: "+minstroke+"-"+maxstroke+" m ( "+minstroke/0.0254+"-"+maxstroke/0.0254+" in )")
    %disp("Axis "+(i+1)+" ideal stroke length: "+idealstroke+" m ( "+idealstroke/0.0254+" in )")
    if minstroke>maxstroke
        %disp("Axis "+(i+1)+" Stroke length not possible, min greater than max")
        score = score - 1 - strokePenalty*(minstroke-maxstroke);
    else
        if minstroke > actuatorDesiredStroke(i)
            score = score - 1 - strokePenalty*(minstroke-actuatorDesiredStroke(i));
        elseif maxstroke < actuatorDesiredStroke(i)
            score = score - 1 - strokePenalty*(actuatorDesiredStroke(i)-maxstroke);
        end
    end
    
    %disp("Axis "+(i+1)+" max force = "+max(abs(force_list(:,i)))+" N ( "+(0.224809*max(abs(force_list(:,i))))+" lbf )")
    %disp("Axis "+(i+1)+" max joint force = "+max(abs(joint_force_list(:,i)))+" N ( "+(0.224809*max(abs(joint_force_list(:,i))))+" lbf )")
    
    maxActForce = max(abs(force_list(:,i)));
    if maxActForce > maxForce
        score = score - 1 - overForcePenalty*(maxActForce-maxForce);
    end
    
end

stowedSize = min(maxX-minX);
stowedSize = max(stowedSize-minStowedSize,0);
score = score - stowedSizePenalty*stowedSize;

if length(endPosList) > 1
    j = boundary(endPosList,1);
    A = area(polyshape(endPosList(j,1),endPosList(j,2)*2));
    score = score + workspaceReward*A;
else
    score = score - 1000;
end

sumAngleRange = sum(angleMaxs-angleMins);
score = score + angleRangeReward*sumAngleRange;

for i = 1:length(arm)-1
    if i > 1
        if arm(i).AB_a < 0
            score = score - leverLengthPenalty*max(abs(arm(i).AB_a)-0.1 ,0);
        elseif arm(i).AB_a > arm(i-1).length
            score = score - leverLengthPenalty*max(abs(arm(i).AB_a-arm(i-1).length/2)-0.1,0);
        end
        
        score = score - leverLengthPenalty*max(abs(arm(i).AB_r)-0.1,0);
    else
        score = score - leverLengthPenalty*max((arm(i).AB_a),0);
        score = score - leverLengthPenalty*max((-arm(i).AB_r),0);
    end
    
    
    if arm(i).AT_a < 0
        score = score - leverLengthPenalty*max(abs(arm(i).AT_a)-0.1,0);
    elseif arm(i).AT_a > arm(i).length
        score = score - leverLengthPenalty*max(abs(arm(i).AT_a-arm(i).length/2)-0.1,0);
    end
    score = score - leverLengthPenalty*max(abs(arm(i).AT_r)-0.1,0);
end

if arm(3).length+arm(2).length > arm(1).length
    score = score - 100*(arm(3).length+arm(2).length-(arm(1).length));
end

if arm(3).length>arm(2).length
    score = score - 100*(arm(3).length-(arm(2).length));
end


score = score*-1;

warning('off','MATLAB:polyshape:repairedBySimplify')
end

