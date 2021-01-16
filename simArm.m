function [angle_list,moment_list,force_list,joint_force_list,actLength_list,achievableAngle_list,endPosList,minX,maxX] = simArm(arm,angleMins,angleMaxs,runs,maxForce)

%Simulation mode (0 = Grid, 1 = Monte Carlo, 2 = Hybrid)
mode = 2; 

monteCarloPoints = round(runs/2); %Number of points to randomly generate for Monte Carlo simulations
gridRuns = round(runs/2);
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
    rng('default')
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
joint_force_list = zeros(length(angles),length(angleMins));
actLength_list = zeros(length(angles),length(angleMins));
achievableAngle_list = zeros(length(angles),length(angleMins));
endPosList = zeros(length(angles),2);

%% Simulation
tic

totNumSims = length(angles);
numSims = 0;
prevNumSims = 0;

numAchievable = 0;

minX = repelem(9999999,length(angles));
maxX = repelem(-9999999,length(angles));
j = 1;
for angle = angles
    numSims = numSims + 1;
    
    angleT = transpose(angle);
    %angle_list = [angle_list; angle];

    [arm1,actLengths] = forwardKinematics(arm, angleT);
    actLength_list(numSims,:) = actLengths;

    %moments = calcMoments(arm1);
    [forces,moments,jointForces] = calcActuatorForces(arm1);

    moment_list(numSims,:) = moments;
    force_list(numSims,:) = forces;
    joint_force_list(numSims,:) = jointForces;
   
    if max(abs(forces))<maxForce && ~checkIntersections(arm1)
        numAchievable = numAchievable + 1;
        achievableAngle_list(numAchievable,:) = angleT;
        endPos = [arm1(end).T_x arm1(end).T_y];
        endPosList(numAchievable,:) = endPos;
    end
    
    for i=1:length(arm1)-1
        thisMinX = min([arm1(i).B_x arm1(i).T_x arm1(i).AB_x arm1(i).AT_x]);
        if thisMinX < minX(j)
            minX(j) = thisMinX;
        end
        
        thisMaxX = max([arm1(i).B_x arm1(i).T_x arm1(i).AB_x arm1(i).AT_x]);
        if thisMaxX > maxX(j)
            maxX(j) = thisMaxX;
        end
    end
    
    axis2.B_x = 0;
    axis2.B_y = 0;
    axis2.T_x = 0;
    axis2.T_y = 0;
    axis2.AB_x = 0;
    axis2.AB_y = 0;
    axis2.AT_x = 0;
    axis2.AT_y = 0;
    
    if toc > 10
        timeRemaining = (totNumSims-numSims)/((numSims-prevNumSims)/toc);
        pctComplete = 100*(numSims/totNumSims);
        disp("Simulation in progress "+pctComplete+"% ... Time remaining: "+timeRemaining+" s")
        
        prevNumSims = numSims;
        tic
    end
    
    j = j + 1;
end

achievableAngle_list(numAchievable+1:end,:) = [];
endPosList(numAchievable+1:end,:) = [];



end

