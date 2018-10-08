function result = isCollisionFree(q_start, q_goal, map)
    %ISCOLLISIONFREE Summary of this function goes here
    %   Detailed explanation goes here
    tolerance = 3;
    dist = sqrt((q_goal(1) - q_start(1))^2 + (q_goal(2) - q_start(2))^2);
    
    if dist <= tolerance
        result = 1;
    else
        q_middle = round((q_start + q_goal)/2);
        
        if map(q_middle(2), q_middle(1), 1) < 255
            result = 0;
        else
            left = isCollisionFree(q_start, q_middle, map);
            right = isCollisionFree(q_middle, q_goal, map);
            result = left && right;
        end
        
    end
    
end

