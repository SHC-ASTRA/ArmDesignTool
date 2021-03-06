function drawArm(arm)
hold on
grid on

colors     = ["#eb4034" "#4542f5" "#63f542" "#f542ef" "#f5ef42" "#42ecf5"];
dullcolors = ["#F5A09A" "#A2A1FA" "#B1FAA1" "#FAA1F7" "#FAF7A1" "#A1F5FA"];

for i = 1:length(arm)
    if arm(i).length > 0
        plot([arm(i).B_x arm(i).T_x], [arm(i).B_y arm(i).T_y], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
        plot([arm(i).B_x], [arm(i).B_y],"o", "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
        armVec = [arm(i).T_x-arm(i).B_x arm(i).T_y-arm(i).B_y];
        armVecUnit = armVec/norm(armVec);
        actTipVec = [arm(i).AT_x-arm(i).B_x arm(i).AT_y-arm(i).B_y]; 
        actTipArmComponent = dot(armVecUnit,actTipVec)*armVecUnit;
        %plot([arm(i).B_x arm(i).AT_x], [arm(i).B_y arm(i).AT_y], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
        plot([arm(i).B_x arm(i).B_x+actTipArmComponent(1)], [arm(i).B_y arm(i).B_y+actTipArmComponent(2)], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
        plot([arm(i).B_x+actTipArmComponent(1) arm(i).AT_x], [arm(i).B_y+actTipArmComponent(2) arm(i).AT_y], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
        plot([arm(i).AB_x arm(i).AT_x], [arm(i).AB_y arm(i).AT_y],':', "Color", colors(mod(i-1,6)+1),'LineWidth',3) 
        if arm(i+1).length > 0
            %plot([arm(i).B_x arm(i+1).AB_x], [arm(i).B_y arm(i+1).AB_y], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
            actBaseVec = [arm(i+1).AB_x-arm(i).B_x, arm(i+1).AB_y-arm(i).B_y];
            actBaseArmComponent = dot(armVecUnit,actBaseVec)*armVecUnit;
            plot([arm(i).B_x arm(i).B_x+actBaseArmComponent(1)], [arm(i).B_y arm(i).B_y+actBaseArmComponent(2)], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
            plot([arm(i).B_x+actBaseArmComponent(1) arm(i+1).AB_x], [arm(i).B_y+actBaseArmComponent(2) arm(i+1).AB_y], "Color", dullcolors(mod(i-1,6)+1),'LineWidth',2)
        end
    end
end
xlim([-2 2])
axis equal
end

