clear;
clc;

%% Map read

map = imread('bitmap.png');
[height, width, ~] = size(map);

%q_goal = [722, 78];
q_goal = [90, 62];

%% Attractive Potential

U_att = zeros(height, width);

zeta = 0.005;
d_star = 2;

for y = 1:height
    for x = 1:width
        
        d = sqrt((x - q_goal(1))^2 + (y - q_goal(2))^2);
        
        if (d <= d_star)
            U_att(y, x) = 0.5 * zeta * (d^2);
        else
            U_att(y, x) = (d_star * zeta * d) - (0.5 * zeta * (d_star^2));
        end
        
    end
end

[X, Y] = meshgrid(1:width, 1:height);

figure(1);
mesh(X, Y, U_att);

%% Repulsive Potential

Q_star = 50;
n = 300;

U_rep = zeros(height, width);

for y = 1:height
    for x = 1:width
        
        d = Q_star;
        
        % Looking surrounds for obstacles within range
        rangey = [y - (Q_star/2), y + (Q_star/2)];
        rangex = [x - (Q_star/2), x + (Q_star/2)];
        
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
        
        for yO = rangey(1):rangey(2)
            for xO = rangex(1):rangex(2)
                
                % Checking if [xO, yO] is an obstacle
                if (map(yO, xO, 1) < 255)
                    d_i = sqrt((x - xO)^2 + (y - yO)^2);
                    
                    % We are looking for the closest obstacle
                    if (d_i < d)
                        d = d_i;
                    end
                    
                end                
            end
        end
        
        U_rep(y, x) = 0.5 * n * ((1/d) - (1/Q_star))^2;
        
        % Saturation
        sat = 10;
        if (U_rep(y, x) > sat)
            U_rep(y, x) = sat;
        end 
        
    end
end

figure(2);
mesh(X, Y, U_rep);

%% Potential Function

U = U_att + U_rep;

figure(3);
mesh(X, Y, U);