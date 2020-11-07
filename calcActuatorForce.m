function [force,moment,jointForce] = calcActuatorForce(arm,axisNum)
[moment,loadWeight] = calcMoment(arm,axisNum);
ActuatorVec = [arm(axisNum).AT_x-arm(axisNum).AB_x arm(axisNum).AT_y-arm(axisNum).AB_y];
LeverVec = [arm(axisNum).AT_x-arm(axisNum).B_x arm(axisNum).AT_y-arm(axisNum).B_y];

ActuatorVecNorm = ActuatorVec / norm(ActuatorVec);
LeverVecNorm = LeverVec / norm(LeverVec);

Angle = acos(dot(ActuatorVecNorm,LeverVecNorm));
force = moment/(norm(LeverVec*sin(Angle)));

actuatorForceVec = ActuatorVecNorm*force;
weightForceVec = [0,-loadWeight];
jointForce = norm(-(actuatorForceVec+weightForceVec));

end
