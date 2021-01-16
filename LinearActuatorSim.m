%% Arm Definition
scalefactor = 0.7;

axis2.length = 0.8*scalefactor;                 %Length of segment (m)
axis2.originMass = 0;               %Mass of segment concentrated at base (kg)
axis2.lengthMass = 3+3.1*0.453592;  %Mass of segment concentrated at midpoint (kg)
axis2.AB_a = -0.55*scalefactor;                 %Actuator base anchor parallel offset (m)
axis2.AB_r = -0.17*scalefactor;                  %Actuator base anchor perpendicular offset (m)
axis2.AT_a = -0.05*scalefactor;                %Actuator tip anchor parallel offset (m)
axis2.AT_r = -0.17*scalefactor;                  %Actuator tip anchor perpendicular offset (m)
axis2.angle = 0;                    % This and all remaining values are IK outputs
axis2.B_x = 0;
axis2.B_y = 0;
axis2.T_x = 0;
axis2.T_y = 0;
axis2.AB_x = 0;
axis2.AB_y = 0;
axis2.AT_x = 0;
axis2.AT_y = 0;

axis3.length = 0.6*scalefactor;
axis3.originMass = 0;
axis3.lengthMass = 3+3.1*0.453592;
axis3.AB_a = 0.2*scalefactor;
axis3.AB_r = 0.10*scalefactor;
axis3.AT_a = 0.15*scalefactor;
axis3.AT_r = -0.075*scalefactor;
axis3.angle = 0;
axis3.B_x = 0;
axis3.B_y = 0;
axis3.T_x = 0;
axis3.T_y = 0;
axis3.AB_x = 0;
axis3.AB_y = 0;
axis3.AT_x = 0;
axis3.AT_y = 0;
    
axis4.length = 0.2*scalefactor;
axis4.originMass = 0;
axis4.lengthMass = 3;
axis4.AB_a = 0.185;
axis4.AB_r = 0.1;
axis4.AT_a = 0.004;
axis4.AT_r = 0.04;
axis4.angle = 0;
axis4.B_x = 0;
axis4.B_y = 0;
axis4.T_x = 0;
axis4.T_y = 0;
axis4.AB_x = 0;
axis4.AB_y = 0;
axis4.AT_x = 0;
axis4.AT_y = 0;

payload.length = 0; % Last axis must have zero length to signify payload
payload.originMass = 5; % Mass of payload
payload.lengthMass = 0; % All other values ignored
payload.AB_a = 0;
payload.AB_r = 0;
payload.AT_a = 0;
payload.AT_r = 0;
payload.angle = 0;
payload.B_x = 0;
payload.B_y = 0;
payload.T_x = 0;
payload.T_y = 0;
payload.AB_x = 0;
payload.AB_y = 0;
payload.AT_x = 0;
payload.AT_y = 0;

% Arm array is made of any number of axes, with a payload (zero length axis) at the end
% (payload is required)
arm = [axis2 axis3 axis4 payload];

%% Variables
% Angle limits (Length must equal number of non-payload axes)
angleMins = [-15,-140,-90];
angleMaxs = [90,-40,45];

% Actuator Specs
%(currently from Progressive Automation PA-09)
safetyFactor = 1.25;
actuatorMaxForce = 330*4.44822; %Newtons
maxForce = actuatorMaxForce/safetyFactor; %Newtons
actuatorFixedLength = 4.53*0.0254; %Meters

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
    
    [maxForceForAxis,maxForceIndex] = max(abs(force_list(:,i)));
    
    disp("Axis "+(i+1)+" max force = "+maxForceForAxis+" N ( "+(0.224809*maxForceForAxis)+" lbf )")
    disp("Axis "+(i+1)+" max joint force = "+max(abs(joint_force_list(:,i)))+" N ( "+(0.224809*max(abs(joint_force_list(:,i))))+" lbf )")
    
    disp("Joint angles at max force: Axis 2 = "+angle_list(maxForceIndex,1)+" degrees, Axis 3 = "+angle_list(maxForceIndex,2)+" degrees, Axis 4 = "+angle_list(maxForceIndex,3)+" degrees")
    disp("Joint actuator lengths at max force: Axis 2 = "+actLength_list(maxForceIndex,1)+" m, Axis 3 = "+actLength_list(maxForceIndex,2)+" m, Axis 4 = "+actLength_list(maxForceIndex,3)+" m")

end

figure(1);
hold on
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


%% surface
% Only supports 3 axes

% figure(4);
% k = boundary(achievableAngle_list,1);
% %plot3(achievableAngle_list(:,1)',achievableAngle_list(:,2)',achievableAngle_list(:,3)',".")
% %hold on
% trisurf(k,achievableAngle_list(:,1),achievableAngle_list(:,2),achievableAngle_list(:,3),'FaceColor','red','FaceAlpha',0.1,'edgealpha',0)