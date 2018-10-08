% Glauber Rodrigues Leite

clear;
clc;

epsilon = 10;
map = imread('bitmap.png');
map_clean = map;
[height, width, ~] = size(map);

robot_size = 10;

q_goal = [722, 78];
%q_goal = [90, 62];

l = 1000;

q_start = [width/2, height/2];

map = insertShape(map,'FilledCircle',[q_goal(1) q_goal(2) robot_size], 'Color', 'green');
map = insertShape(map,'FilledCircle',[q_start(1) q_start(2) robot_size], 'Color', 'green');

T_1.Vertices = q_start;
T_1.Edges = [];
T_2.Vertices = q_goal;
T_2.Edges = [];

epoch = 1;
merged = false;

while epoch < l
    % Computing random configuration within Q_free space
    % Using Normal Random Distribution
    q_rand = round([rand*width, rand*height]);

    while ~any(q_rand) || map(q_rand(2), q_rand(1), 1) < 255
        q_rand = round([rand*width, rand*height]);
    end

    [q_new_1, T_1, q_near_1] = extend_rrt(q_rand, T_1, map_clean);
    
    if q_new_1(1) ~= -1
        [q_new_2, T_2, q_near_2] = extend_rrt(q_new_1, T_2, map_clean);
        
        if norm(q_new_1 - q_new_2) <= epsilon
            merged = true;
            epoch = l;
        else    
            % Swapping T_1 and T_2
            aux = T_1;
            T_1 = T_2;
            T_2 = aux;
        end
        
    end
            
    something = false;
    if q_new_1(1) ~= -1

        % Making changes on image
        map = insertShape(map,'FilledCircle',[q_new_1 robot_size], 'Color', 'red');
        map = insertShape(map,'Line',[q_new_1 q_near_1], 'Color', 'red', 'LineWidth', 5);
        image(map)
        
        something = true;
    end
    
    if q_new_2(1) ~= -1

        % Making changes on image
        map = insertShape(map,'FilledCircle',[q_new_2 robot_size], 'Color', 'red');
        map = insertShape(map,'Line',[q_new_2 q_near_2], 'Color', 'red', 'LineWidth', 5);
        image(map)
        
        something = true;
    end
    
    if something
        pause
    end
    
    epoch = epoch + 1;
end