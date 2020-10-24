%% Arm Definition

axis1.length = 0.8;                 %Length of segment (m)
axis1.originMass = 0;               %Mass of segment concentrated at base (kg)
axis1.lengthMass = 3+4.2*0.453592;  %Mass of segment concentrated at midpoint (kg)
axis1.AB_a = -0.55;                 %Actuator base anchor parallel offset (m)
axis1.AB_r = -0.17;                  %Actuator base anchor perpendicular offset (m)
axis1.AT_a = -0.05;                %Actuator tip anchor parallel offset (m)
axis1.AT_r = -0.17;                  %Actuator tip anchor perpendicular offset (m)
axis1.angle = 0;                    % This and all remaining values are IK outputs
axis1.B_x = 0;
axis1.B_y = 0;
axis1.T_x = 0;
axis1.T_y = 0;
axis1.AB_x = 0;
axis1.AB_y = 0;
axis1.AT_x = 0;
axis1.AT_y = 0;

axis2.length = 0.6;
axis2.originMass = 0;
axis2.lengthMass = 3+4.2*0.453592;
axis2.AB_a = 0.2;
axis2.AB_r = 0.10;
axis2.AT_a = 0.15;
axis2.AT_r = -0.075;
axis2.angle = 0;
axis2.B_x = 0;
axis2.B_y = 0;
axis2.T_x = 0;
axis2.T_y = 0;
axis2.AB_x = 0;
axis2.AB_y = 0;
axis2.AT_x = 0;
axis2.AT_y = 0;
    
axis3.length = 0.2;
axis3.originMass = 0;
axis3.lengthMass = 3;
axis3.AB_a = 0.1;
axis3.AB_r = 0.1;
axis3.AT_a = 0;
axis3.AT_r = 0.1;
axis3.angle = 0;
axis3.B_x = 0;
axis3.B_y = 0;
axis3.T_x = 0;
axis3.T_y = 0;
axis3.AB_x = 0;
axis3.AB_y = 0;
axis3.AT_x = 0;
axis3.AT_y = 0;

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
arm = [axis1 axis2 axis3 payload]; 

%% Variables
% Angle limits (Length must equal number of non-payload axes)
angleMins = [-15,-140,-85];
angleMaxs = [90,-40,70];

% Actuator Specs
%(currently from Progressive Automation PA-09)
safetyFactor = 1.25;
actuatorMaxForce = 330*4.44822; %Newtons
maxForce = actuatorMaxForce/safetyFactor; %Newtons
actuatorFixedLength = 5.71*0.0254; %Meters

%Simulation mode (0 = Grid, 1 = Monte Carlo, 2 = Hybrid)
mode = 2; 

monteCarloPoints = 25000; %Number of points to randomly generate for Monte Carlo simulations
gridRuns = 25000;
gridPoints = floor(nthroot(gridRuns,length(angleMins))); % Number of grid points per axis for grid simulations
% Note: number of simulations is equal to the gridpoints to the power of
% the number of axes, large numbers of axes or gridpoints will increase the
% simulation time exponentially

%% Setup

if mode == 0 || mode == 2
    elements = [];
    anglesG = [];

    for i = 1:length(angleMins)
        elements(i,:) = linspace(angleMins(i),angleMaxs(i),gridPoints);
    end

    if length(angleMins) == 1 
        [a1] = ndgrid(elements(1,:));
        anglesG = a1;
    elseif length(angleMins) == 2 
        [a1,a2] = ndgrid(elements(1,:),elements(2,:));
        anglesG = [reshape(a1,1,[]); reshape(a2,1,[])];
    elseif length(angleMins) == 3 
        [a1,a2,a3] = ndgrid(elements(1,:),elements(2,:),elements(3,:));
        anglesG = [reshape(a1,1,[]); reshape(a2,1,[]); reshape(a3,1,[])];
    elseif length(angleMins) == 4 
        [a1,a2,a3,a4] = ndgrid(elements(1,:),elements(2,:),elements(3,:),elements(4,:));
        anglesG = [reshape(a1,1,[]); reshape(a2,1,[]); reshape(a3,1,[]); reshape(a4,1,[])];
    elseif length(angleMins) == 5 
        [a1,a2,a3,a4,a5] = ndgrid(elements(1,:),elements(2,:),elements(3,:),elements(4,:),elements(5,:));
        anglesG = [reshape(a1,1,[]); reshape(a2,1,[]); reshape(a3,1,[]); reshape(a4,1,[]); reshape(a5,1,[])];
    else
        error("Unimplemented number of axes, bug Thomas to add more");
    end
end

if mode == 1 || mode == 2
    anglesM = [];
    for i = 1:length(angleMins)
        a = angleMins(i);
        b = angleMaxs(i);
        r = (b-a).*rand(monteCarloPoints,1) + a;            
        anglesM(i,:) = r; 
    end
end

if mode == 0
    angles = anglesG;
elseif mode == 1
    angles = anglesM;
elseif mode == 2
    angles = [anglesG anglesM];
else 
    angles = zeroes(length(angleMins));
end

angle_list = angles';
moment_list = zeros(length(angles),length(angleMins));
force_list = zeros(length(angles),length(angleMins));
actLength_list = zeros(length(angles),length(angleMins));
achievableAngle_list = zeros(length(angles),length(angleMins));
endPosList = zeros(length(angles),2);

%% Simulation
tic

totNumSims = length(angles);
numSims = 0;
prevNumSims = 0;

numAchievable = 0;
for angle = angles
    numSims = numSims + 1;
    
    angle = transpose(angle);
    %angle_list = [angle_list; angle];

    [arm1,actLengths] = forwardKinematics(arm, angle);
    actLength_list(numSims,:) = actLengths;

    %moments = calcMoments(arm1);
    [forces,moments] = calcActuatorForces(arm1);

    moment_list(numSims,:) = moments;
    force_list(numSims,:) = forces;
   
    if max(abs(forces))<maxForce && ~checkIntersections(arm1)
        numAchievable = numAchievable + 1;
        achievableAngle_list(numAchievable,:) = angle;
        endPos = [arm1(end).T_x arm1(end).T_y];
        endPosList(numAchievable,:) = endPos;
    end
    
    if toc > 10
        timeRemaining = (totNumSims-numSims)/((numSims-prevNumSims)/toc);
        pctComplete = 100*(numSims/totNumSims);
        disp("Simulation in progress "+pctComplete+"% ... Time remaining: "+timeRemaining+" s")
        
        prevNumSims = numSims;
        tic
    end
end

achievableAngle_list(numAchievable+1:end,:) = [];
endPosList(numAchievable+1:end,:) = [];

%% Outputs
figure(2);
tiledlayout('flow') 
for i = 1:length(angleMins)
    nexttile
    plot(angle_list(:,i),actLength_list(:,i),"g.")
    title("Actuator Length Axis "+i)
    xlabel("Joint Angle (deg)");
    ylabel("Actuator Length (m)");
    
    disp("Axis "+i+" min extension = "+min(actLength_list(:,i))+" m ( "+(min(actLength_list(:,i))/0.0254)+" in )")
    disp("Axis "+i+" max extension = "+max(actLength_list(:,i))+" m ( "+(max(actLength_list(:,i))/0.0254)+" in )")
    
    %fixed+stroke = min
    %stroke = min-fixed
    
    %fixed+2stroke = max
    %2stroke = max-fixed
    %stroke = (max-fixed)/2
    
    maxstroke = min(actLength_list(:,i))-actuatorFixedLength;
    minstroke = (max(actLength_list(:,i))-actuatorFixedLength)/2;
    idealstroke = (minstroke+maxstroke)/2;
    disp("Axis "+i+" stroke length range: "+minstroke+"-"+maxstroke+" m ( "+minstroke/0.0254+"-"+maxstroke/0.0254+" in )")
    disp("Axis "+i+" ideal stroke length: "+idealstroke+" m ( "+idealstroke/0.0254+" in )")
    if minstroke>maxstroke
        disp("Axis "+i+" Stroke length not possible, min greater than max")
    end
    
end

figure(3);
tiledlayout('flow') 
for i = 1:length(angleMins)
    nexttile
    plot(angle_list(:,i),abs(force_list(:,i)),"b.")
    title("Actuator Forces Axis "+i)
    xlabel("Joint Angle (deg)");
    ylabel("Actuator Force (N)");
    hold on
    
    plot([angleMins(i) angleMaxs(i)],[maxForce maxForce],"r:");
    %plot([angleMins(i) angleMaxs(i)],[-maxForce -maxForce],"r:");
    
    disp("Axis "+i+" max force = "+max(abs(force_list(:,i)))+" N ( "+(0.224809*max(abs(force_list(:,i))))+" lbf )")
end

figure(1);
hold on
% [arm1,~] = forwardKinematics(arm, [angleMaxs(1) angleMins(2) -angleMaxs(1)-angleMins(2)-90]);
[arm1,~] = forwardKinematics(arm, (angleMaxs+angleMins)/2);
drawArm(arm1)
% [arm1,~] = forwardKinematics(arm, angleMaxs);
% drawArm(arm1)
% [arm1,~] = forwardKinematics(arm, angleMins);
% drawArm(arm1)
j = boundary(endPosList,1);
plot(endPosList(j,1),endPosList(j,2))


%% surface
% Only supports 3 axes

% figure(4);
% k = boundary(achievableAngle_list,1);
% %plot3(achievableAngle_list(:,1)',achievableAngle_list(:,2)',achievableAngle_list(:,3)',".")
% %hold on
% trisurf(k,achievableAngle_list(:,1),achievableAngle_list(:,2),achievableAngle_list(:,3),'FaceColor','red','FaceAlpha',0.1,'edgealpha',0)