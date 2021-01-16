options = optimoptions('particleswarm','Display','iter','UseParallel',true,'FunctionTolerance',1e-6,'PlotFcn',{@pswplotbestf,@bestArmPlotFcn});

nvar = 20;
lb = repelem(0,nvar);
ub = repelem(1,nvar);
[x,fval,exitflag,output] = particleswarm(@evalArm,nvar,lb,ub,options)

%%

%x = [0.0000    0.3251    0.5060    0.9132    0.5326    0.4943    0.7422    0.5319    0.7339    0.4872    0.9339    0.4860    0.1638    0.5191 0.0991    0.0371    0.0413    0.6854    0.0172    0.9969];

%% Arm Definition

safetyFactor = 1.25;
actuatorMaxForce = 330*4.44822; %Newtons
maxForce = actuatorMaxForce/safetyFactor; %Newtons
actuatorFixedLength = 4.53*0.0254; %Meters
actuatorDesiredStroke = [8*0.0254 8*0.0254 6*0.0254]; %Meters

[arm, angleMins, angleMaxs] = x2Arm(x);

%% Run

[angle_list,moment_list,force_list,joint_force_list,actLength_list,achievableAngle_list,endPosList,minX,maxX] = simArm(arm,angleMins,angleMaxs,50000,maxForce);

%% Outputs
figure(2);
tiledlayout('flow') 
for i = 1:length(angleMins)
    nexttile
    plot(angle_list(:,i),actLength_list(:,i),"g.")
    title("Actuator Length Axis "+(i+1))
    xlabel("Joint Angle (deg)");
    ylabel("Actuator Length (m)");
    
    [sortedAngles,sortIdx] = sort(angle_list(:,i));
    sortedLengths = actLength_list(sortIdx,i);
    [sortedAngles,uniqueIdx,~] = unique(sortedAngles);
    sortedLengths = sortedLengths(uniqueIdx);
    d_Lengths = diff(sortedLengths)./diff(sortedAngles)*360;
    
    
    disp("Axis "+(i+1)+" min extension = "+min(actLength_list(:,i))+" m ( "+(min(actLength_list(:,i))/0.0254)+" in )")
    disp("Axis "+(i+1)+" max extension = "+max(actLength_list(:,i))+" m ( "+(max(actLength_list(:,i))/0.0254)+" in )")
    disp("Axis "+(i+1)+" max slope = "+max(abs(d_Lengths))+" m/rev ( "+(max(abs(d_Lengths))/0.0254)+" in/rev )")
    
    %fixed+stroke = min
    %stroke = min-fixed
    
    %fixed+2stroke = max
    %2stroke = max-fixed
    %stroke = (max-fixed)/2
    
    maxstroke = min(actLength_list(:,i))-actuatorFixedLength;
    minstroke = (max(actLength_list(:,i))-actuatorFixedLength)/2;
    idealstroke = (minstroke+maxstroke)/2;
    disp("Axis "+(i+1)+" stroke length range: "+minstroke+"-"+maxstroke+" m ( "+minstroke/0.0254+"-"+maxstroke/0.0254+" in )")
    disp("Axis "+(i+1)+" ideal stroke length: "+idealstroke+" m ( "+idealstroke/0.0254+" in )")
    if minstroke>maxstroke
        disp("Axis "+(i+1)+" Stroke length not possible, min greater than max")
    end
    
end

figure(3);
tiledlayout('flow') 
for i = 1:length(angleMins)
    nexttile
    plot(angle_list(:,i),abs(force_list(:,i)),"b.")
    title("Actuator Forces Axis "+(i+1))
    xlabel("Joint Angle (deg)");
    ylabel("Actuator Force (N)");
    hold on
    
    plot([angleMins(i) angleMaxs(i)],[maxForce maxForce],"r:");
    %plot([angleMins(i) angleMaxs(i)],[-maxForce -maxForce],"r:");
    
    disp("Axis "+(i+1)+" max force = "+max(abs(force_list(:,i)))+" N ( "+(0.224809*max(abs(force_list(:,i))))+" lbf )")
    disp("Axis "+(i+1)+" max joint force = "+max(abs(joint_force_list(:,i)))+" N ( "+(0.224809*max(abs(joint_force_list(:,i))))+" lbf )")
end

figure(1);
hold on
cla
j = boundary(endPosList,1);
plot(endPosList(j,1),endPosList(j,2),"c")
[x_c,y_c] = centroid(polyshape(endPosList(j,1),endPosList(j,2)));
[minDist,closestIdx] = min(sqrt((x_c-endPosList(:,1)).^2 + (y_c-endPosList(:,2)).^2 + (0.0001)*(-90-(achievableAngle_list(:,1)+achievableAngle_list(:,2)+achievableAngle_list(:,3))).^2));

% [arm1,~] = forwardKinematics(arm, [angleMaxs(1) angleMins(2) -angleMaxs(1)-angleMins(2)-90]);
[arm1,~] = forwardKinematics(arm, achievableAngle_list(closestIdx,:));
drawArm(arm1)
% [arm1,~] = forwardKinematics(arm, angleMaxs);
% drawArm(arm1)
% [arm1,~] = forwardKinematics(arm, angleMins);
% drawArm(arm1)


plot(x_c,y_c,"cx")

stowedSize = min(maxX-minX)


%% surface
% Only supports 3 axes

% figure(4);
% k = boundary(achievableAngle_list,1);
% %plot3(achievableAngle_list(:,1)',achievableAngle_list(:,2)',achievableAngle_list(:,3)',".")
% %hold on
% trisurf(k,achievableAngle_list(:,1),achievableAngle_list(:,2),achievableAngle_list(:,3),'FaceColor','red','FaceAlpha',0.1,'edgealpha',0)