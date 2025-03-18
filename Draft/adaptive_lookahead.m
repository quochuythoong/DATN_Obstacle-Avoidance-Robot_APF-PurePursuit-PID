%% Adaptive Lookahead Pure Pursuit Simulation

% Define the path (example: sinusoidal curve)
x = linspace(0, 100, 1000);
y = 10 * sin(x/10);
path = [x; y]'; % 1000 x 2 matrix

% Compute approximate curvature along the path using finite differences
dx = gradient(x);
dy = gradient(y);
ddx = gradient(dx);
ddy = gradient(dy);
curvature = abs(dx.*ddy - dy.*ddx) ./ ((dx.^2 + dy.^2).^(3/2));
curvature(isnan(curvature)) = 0;  % Avoid NaNs

% Parameters
v = 5;         % Constant speed (m/s)
k1 = 0.5;      % Tunable coefficient for speed contribution
k2 = -10;      % Tunable coefficient for curvature contribution (negative to reduce ld in tight curves)
min_ld = 5;    % Minimum lookahead distance (m)

% Initialize car position at the start of the path
car_idx = 1;
car_pos = path(car_idx, :);

% Setup visualization
figure;
plot(x, y, 'k-', 'LineWidth', 2); hold on;
plot(path(:,1), path(:,2), 'b.'); % Display waypoints
hCar = plot(car_pos(1), car_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hLA = plot(car_pos(1), car_pos(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Lookahead point
title('Adaptive Lookahead Pure Pursuit Simulation');
xlabel('X (m)'); ylabel('Y (m)');
legend('Path', 'Waypoints', 'Car', 'Lookahead');

% Simulation loop
for t = 1:500
    % Get current curvature at car position (approximate using path index)
    if car_idx < length(curvature)
        current_curvature = curvature(car_idx);
    else
        current_curvature = curvature(end);
    end
    
    % Calculate adaptive lookahead distance (ld = k1*v + k2*kappa)
    ld = k1 * v + k2 * current_curvature;
    if ld < min_ld
        ld = min_ld;
    end

    % Find the lookahead point along the path
    dist_accum = 0;
    lookahead_idx = car_idx;
    while lookahead_idx < size(path,1) && dist_accum < ld
        dist_accum = dist_accum + norm(path(lookahead_idx+1,:) - path(lookahead_idx,:));
        lookahead_idx = lookahead_idx + 1;
    end
    if lookahead_idx > size(path,1)
        lookahead_idx = size(path,1);
    end
    la_point = path(lookahead_idx, :);

    % Pure Pursuit step: Move a small step toward the lookahead point
    direction = la_point - car_pos;
    if norm(direction) ~= 0
        direction = direction / norm(direction);
    end
    dt = 0.1; % time step (s)
    step = v * dt; 
    car_pos = car_pos + step * direction;
    
    % Update car index by finding the nearest point on the path
    [~, car_idx] = min(sum((path - car_pos).^2, 2));

    % Update visualization
    set(hCar, 'XData', car_pos(1), 'YData', car_pos(2));
    set(hLA, 'XData', la_point(1), 'YData', la_point(2));
    drawnow;
    
    pause(0.05); % Pause for visualization
end
