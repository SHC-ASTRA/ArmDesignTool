function [arm, actLengths] = forwardKinematics(arm, angles)
linkCount = 0;
for axis = arm
    if(axis.length ~= 0)
        linkCount = linkCount + 1;
    end
end

if(linkCount ~= length(angles))
    for axis = arm
       disp(axis);
    end
    error("Length of angle array must equal number of non-zero-length arm segments")
    return
end

angles = deg2rad(angles);



for i = 1:length(arm)
    
    if i > 1
        baseX = arm(i-1).T_x;
        baseY = arm(i-1).T_y;
        prevbaseX = arm(i-1).B_x;
        prevbaseY = arm(i-1).B_y;
        [ABx,ABy] = rotPoint(arm(i).AB_a, arm(i).AB_r,sum(angles(1:(i-1))));
    else
        baseX = 0;
        baseY = 0;
        prevbaseX = 0;
        prevbaseY = 0;
        ABx = arm(i).AB_a;
        ABy = arm(i).AB_r;
    end
    
    if arm(i).length > 0
        [x,y] = rotPoint(arm(i).length,0,sum(angles(1:i)));
        [ATx,ATy] = rotPoint(arm(i).AT_a, arm(i).AT_r,sum(angles(1:i)));
    else
        x = 0;
        y = 0;
        ATx = 0;
        ATy = 0;
    end
    
    arm(i).B_x = baseX;
    arm(i).B_y = baseY;
    arm(i).T_x = x + baseX;
    arm(i).T_y = y + baseY;
    arm(i).AT_x = baseX + ATx;
    arm(i).AT_y = baseY + ATy;
    
    
    arm(i).AB_x = prevbaseX + ABx;
    arm(i).AB_y = prevbaseY + ABy;
    
    if arm(i).length > 0
        actLengths(i) = norm([arm(i).AT_x-arm(i).AB_x arm(i).AT_y-arm(i).AB_y]);
    end
    
end

