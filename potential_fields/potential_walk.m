clear;
clc;

map = imread('bitmap.png');
[height, width, ~] = size(map);


robot_size = 10;

%q_goal = [722, 78];
q_goal = [90, 62];

map = insertShape(map,'FilledCircle',[q_goal(1) q_goal(2) robot_size], 'Color', 'blue');

q_start = [width/2, height/2];
zeta = 0.005;
d_star = 2;
Q_star = 50;
n = 300;
alpha = 1;
tolerance = 0.0001;

q = q_start;

grad_U = 1;
while norm(grad_U) > tolerance
    
    % Calculating gradient   
    
    % grad_U_att
    d = norm(q - q_goal);
    if (d <= d_star)
        grad_U_att = zeta * (q - q_goal);
    else
        grad_U_att = d_star * zeta * (q - q_goal) / d;
    end
    
    % grad_U_rep
    d = Q_star;
    
    rangey = [q(2) - (Q_star/2), q(2) + (Q_star/2)];
    rangex = [q(1) - (Q_star/2), q(1) + (Q_star/2)];

    if (rangey(1) <= 0)
        rangey(1) = 1;
    end
    if (rangex(1) <= 0) 
        rangex(1) = 1;
    end
    if (rangey(2) > height)
        rangey(2) = height;
    end
    if (rangex(2) > width) 
        rangex(2) = width;
    end

    for yO = round(rangey(1)):round(rangey(2))
        for xO = round(rangex(1)):round(rangex(2))

            % Checking if [xO, yO] is an obstacle
            if (map(yO, xO, :) == [0, 0, 0])
                d_i = norm(q - [xO, yO]);

                % We are looking for the closest obstacle
                if (d_i < d)
                    d = d_i;
                    c = [xO, yO];
                end

            end                
        end
    end

    if (d < Q_star)
        grad_d = (q - c)/d;
        grad_U_rep = n * ((1/Q_star) - (1/d)) * (1/(d^2)) * grad_d;
    else
        grad_U_rep = 0;
    end
    
    % Now we can have grad_U
    grad_U = grad_U_att + grad_U_rep;

    % Moving
    if (norm(q - q_goal) > 10)
        alpha = norm(q - q_goal) - (1/d);
    else
        alpha = d;
    end
    q = q + alpha * (-grad_U);
    
    % Making changes on image
    map = insertShape(map,'FilledCircle',[q(1) q(2) robot_size], 'Color', 'red');
    image(map)
    pause
 
end