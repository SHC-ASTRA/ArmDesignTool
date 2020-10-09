function intersect = checkIntersections(arm)
intersect = false;
for i = 1:length(arm)
    if arm(i).length > 0
        for j = i+1:length(arm)
            if abs(i-j)>1
                seg1centerx = (arm(i).B_x+arm(i).T_x)/2;
                seg1centery = (arm(i).B_y+arm(i).T_y)/2;
                seg1len=arm(i).length;
                seg2centerx = (arm(j).B_x+arm(j).T_x)/2;
                seg2centery = (arm(j).B_y+arm(j).T_y)/2;
                seg2len=arm(j).length;
                
                dist = sqrt((seg1centerx-seg2centerx)^2 + (seg1centery-seg2centery)^2);
                
                if dist <= (seg1len+seg2len)/2 
                    [x,~] = polyxpoly([arm(i).B_x arm(i).T_x], [arm(i).B_y arm(i).T_y], [arm(j).B_x arm(j).T_x], [arm(j).B_y arm(j).T_y]);
                    if ~isempty(x)
                        intersect = true;
                    end
                end
            end
        end
    end
end

