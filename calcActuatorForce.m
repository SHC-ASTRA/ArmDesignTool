function [force,moment] = calcActuatorForce(arm,axisNum)
moment = calcMoment(arm,axisNum);
ActuatorVec = [arm(axisNum).AT_x-arm(axisNum).AB_x arm(axisNum).AT_y-arm(axisNum).AB_y];
LeverVec = [arm(axisNum).AT_x-arm(axisNum).B_x arm(axisNum).AT_y-arm(axisNum).B_y];

ActuatorVecNorm = ActuatorVec / norm(ActuatorVec);
LeverVecNorm = LeverVec / norm(LeverVec);

Angle = acos(dot(ActuatorVecNorm,LeverVecNorm));
force = moment/(norm(LeverVec*sin(Angle)));

end
