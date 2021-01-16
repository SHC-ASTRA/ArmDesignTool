function [arm, angleMins, angleMaxs] = x2Arm(x)
totalLength = 1.5;
leverLength = 0.5;

length1 = mapRange(x(1),0.1,totalLength-0.2);
length2 = mapRange(x(2),0.1,totalLength-length1-0.1);
length3 = totalLength-length1-length2;

axis2.length = length1;                 %Length of segment (m)
axis2.originMass = 0;               %Mass of segment concentrated at base (kg)
axis2.lengthMass = 3.5*length1+3.1*0.453592;  %Mass of segment concentrated at midpoint (kg)
axis2.AB_a = mapRange(x(3),-leverLength,leverLength);                 %Actuator base anchor parallel offset (m)
axis2.AB_r = mapRange(x(4),-leverLength,leverLength);                %Actuator base anchor perpendicular offset (m)
axis2.AT_a = mapRange(x(5),-leverLength,leverLength);                %Actuator tip anchor parallel offset (m)
axis2.AT_r = mapRange(x(6),-leverLength,leverLength);                  %Actuator tip anchor perpendicular offset (m)
axis2.angle = 0;                    % This and all remaining values are IK outputs
axis2.B_x = 0;
axis2.B_y = 0;
axis2.T_x = 0;
axis2.T_y = 0;
axis2.AB_x = 0;
axis2.AB_y = 0;
axis2.AT_x = 0;
axis2.AT_y = 0;

axis3.length = length2;
axis3.originMass = 0;
axis3.lengthMass = 3.5*length2+3.1*0.453592;
axis3.AB_a = mapRange(x(7),-leverLength,leverLength);
axis3.AB_r = mapRange(x(8),-leverLength,leverLength);
axis3.AT_a = mapRange(x(9),-leverLength,leverLength);
axis3.AT_r = mapRange(x(10),-leverLength,leverLength);
axis3.angle = 0;
axis3.B_x = 0;
axis3.B_y = 0;
axis3.T_x = 0;
axis3.T_y = 0;
axis3.AB_x = 0;
axis3.AB_y = 0;
axis3.AT_x = 0;
axis3.AT_y = 0;
    
axis4.length = length3;
axis4.originMass = 0;
axis4.lengthMass = 3.5*length3;
axis4.AB_a = mapRange(x(11),-leverLength,leverLength);
axis4.AB_r = mapRange(x(12),-leverLength,leverLength);
axis4.AT_a = mapRange(x(13),-leverLength,leverLength);
axis4.AT_r = mapRange(x(14),-leverLength,leverLength);
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

angle1Range = mapRange(x(15),0,160);
angle2Range = mapRange(x(16),0,160);
angle3Range = mapRange(x(17),0,160);

angle1Off = mapRange(x(18),-45,0);
angle2Off = mapRange(x(19),-180,-135);
angle3Off = mapRange(x(20),-180,-135);

angleMins = [angle1Off,angle2Off,angle3Off];
angleMaxs = [angle1Off+angle1Range,angle2Off+angle2Range,angle3Off+angle3Range];
end

