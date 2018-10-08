function [q_new, T, q_near] = extend_rrt(q, T, map)
    %EXTEND_RRT Summary of this function goes here
    %   Detailed explanation goes here

    % Finding nearest node to q
    
    if size(T.Vertices, 1) == 1
        q_near = T.Vertices;
    else
        q_near = T.Vertices(1, :);
    end
    minDist = sqrt((q(1) - q_near(1))^2 + (q(2) - q_near(2))^2);
    for k=2:size(T.Vertices, 1)
        candidate = T.Vertices(k, :);
        dist = sqrt((q(1) - candidate(1))^2 + (q(2) - candidate(2))^2);
        if (dist < minDist)
           q_near = candidate;
           minDist = dist;
        end
    end
    
    % Setting step_size
    step_size = minDist/10;
    
    % Computing q_new
    q_new = round(q_near + step_size * ((q - q_near)/minDist));
    
    % Checking collision
    if map(q_new(2), q_new(1), 1) == 255 && isCollisionFree(q_near, q_new, map)
        T.Vertices = [T.Vertices; q_new];
        T.Edges = [T.Edges; [q_near, q_new]];
    else
        q_new = [-1, -1];
    end
    
end

