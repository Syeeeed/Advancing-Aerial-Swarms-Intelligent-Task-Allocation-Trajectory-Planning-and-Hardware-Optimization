%% Aerial Swarm Simulation: Task Allocation & Trajectory Planning
% Based on: Advancing Aerial Swarms (Syed Ashque MD Khaled)
% Framework: Task Allocation -> Trajectory Planning -> Execution

clear; clc; close all;

%% 1. Configuration
num_drones = 6;
num_tasks = 8;
env_size = 50; % 3D Environment size [0, env_size]

% Parameters for Trajectory Planning (Artificial Potential Fields)
k_att = 0.5;   % Attractive force gain
k_rep = 100;   % Repulsive force gain
d_min = 3.0;   % Minimum safety distance between drones
max_vel = 1.5; % Maximum velocity
k_wall = 200;  % Repulsive force gain from walls
d_wall = 4.0;  % Safety distance from walls

%% 2. Initialization
% Initialize Drones
drones = struct();
for i = 1:num_drones
    drones(i).id = i;
    drones(i).pos = [rand*10, rand*10, rand*10 + 5]; % Start near origin
    drones(i).vel = [0, 0, 0];
    drones(i).target_id = -1; % Currently no task
    drones(i).path = drones(i).pos;
    drones(i).color = rand(1, 3);
end

% Initialize Tasks (Targets)
tasks = struct();
for j = 1:num_tasks
    tasks(j).id = j;
    tasks(j).pos = [rand*env_size, rand*env_size, rand*15 + 5];
    tasks(j).assigned = false;
    tasks(j).completed = false;
end

%% 3. Main Simulation Loop
figure('Color', 'w', 'Name', 'Aerial Swarm Simulation');
view(3); grid on; axis([0 env_size 0 env_size 0 25]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
hold on;
title('Decentralized Aerial Swarm: Task Allocation & Collision Avoidance');

% Runs indefinitely as long as the figure window is open
while ishandle(gcf)
    cla;
    % Draw Environment Boundaries
    rectangle('Position',[0,0,env_size,env_size], 'EdgeColor', [0.8 0.8 0.8]);
    
    %% A. Decentralized Task Allocation (Simplified Auction Logic)
    for i = 1:num_drones
        % If drone has no target, find the nearest unassigned task
        if drones(i).target_id == -1
            min_dist = inf;
            best_task = -1;
            for j = 1:num_tasks
                if ~tasks(j).assigned && ~tasks(j).completed
                    dist = norm(drones(i).pos - tasks(j).pos);
                    if dist < min_dist
                        min_dist = dist;
                        best_task = j;
                    end
                end
            end
            
            if best_task ~= -1
                drones(i).target_id = best_task;
                tasks(best_task).assigned = true;
                fprintf('Drone %d assigned to Task %d\n', i, best_task);
            end
        end
    end

    %% B. Trajectory Planning & Execution (APF Controller)
    for i = 1:num_drones
        force_total = [0, 0, 0];
        
        if drones(i).target_id ~= -1
            target_pos = tasks(drones(i).target_id).pos;
            dist_to_target = norm(drones(i).pos - target_pos);
            
            % 1. Attractive Force to Target
            f_att = -k_att * (drones(i).pos - target_pos);
            force_total = force_total + f_att;
            
            % Task Completion Check
            if dist_to_target < 1.0
                tasks(drones(i).target_id).completed = true;
                fprintf('Drone %d completed Task %d!\n', i, drones(i).target_id);
                drones(i).target_id = -1;
            end
        end
        
        % 2. Repulsive Force from other Drones (Collision Avoidance)
        for other = 1:num_drones
            if i == other, continue; end
            rel_pos = drones(i).pos - drones(other).pos;
            dist = norm(rel_pos);
            if dist < d_min
                f_rep = k_rep * (1/dist - 1/d_min) * (1/dist^2) * (rel_pos / dist);
                force_total = force_total + f_rep;
            end
        end
        
        % 3. Repulsive Force from Walls (Boundary Constraints)
        for dim = 1:3
            limit_max = env_size;
            if dim == 3, limit_max = 25; end % Z limit
            
            % Lower bound
            dist_low = max(0.1, drones(i).pos(dim));
            if dist_low < d_wall
                force_total(dim) = force_total(dim) + k_wall * (1/dist_low - 1/d_wall) * (1/dist_low^2);
            end
            % Upper bound
            dist_high = max(0.1, limit_max - drones(i).pos(dim));
            if dist_high < d_wall
                force_total(dim) = force_total(dim) - k_wall * (1/dist_high - 1/d_wall) * (1/dist_high^2);
            end
        end
        
        % 4. Update Dynamics
        drones(i).vel = drones(i).vel + force_total * 0.1;
        % Limit velocity
        if norm(drones(i).vel) > max_vel
            drones(i).vel = (drones(i).vel / norm(drones(i).vel)) * max_vel;
        end
        
        drones(i).pos = drones(i).pos + drones(i).vel;
        
        % Hard clamping to ensure drones stay inside frame
        drones(i).pos(1:2) = min(max(drones(i).pos(1:2), 0.5), env_size-0.5);
        drones(i).pos(3) = min(max(drones(i).pos(3), 1.0), 24.5);
        
        drones(i).path = [drones(i).path; drones(i).pos];
        % Keep path length manageable for long-running simulation
        if size(drones(i).path, 1) > 200
            drones(i).path(1, :) = [];
        end
    end

    %% C. Visualization
    % Plot Tasks
    for j = 1:num_tasks
        if ~tasks(j).completed
            plot3(tasks(j).pos(1), tasks(j).pos(2), tasks(j).pos(3), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
            text(tasks(j).pos(1), tasks(j).pos(2), tasks(j).pos(3)+1, sprintf('T%d', j), 'FontSize', 8);
        end
    end
    
    % Plot Drones
    for i = 1:num_drones
        % Plot path
        plot3(drones(i).path(:,1), drones(i).path(:,2), drones(i).path(:,3), 'Color', [drones(i).color 0.3]);
        % Plot drone as a sphere/point
        plot3(drones(i).pos(1), drones(i).pos(2), drones(i).pos(3), 'o', ...
            'MarkerSize', 8, 'MarkerFaceColor', drones(i).color, 'MarkerEdgeColor', 'k');
        % Label
        text(drones(i).pos(1), drones(i).pos(2), drones(i).pos(3)+1.5, sprintf('D%d', i), ...
            'Color', drones(i).color, 'FontWeight', 'bold');
        
        % Line to target
        if drones(i).target_id ~= -1
            t_pos = tasks(drones(i).target_id).pos;
            line([drones(i).pos(1) t_pos(1)], [drones(i).pos(2) t_pos(2)], [drones(i).pos(3) t_pos(3)], ...
                'Color', [drones(i).color 0.2], 'LineStyle', '--');
        end
    end
    
    drawnow;
    pause(0.01);
    
    % If all tasks completed, regenerate new ones (Infinite Loop)
    if all([tasks.completed])
        fprintf('All tasks completed! Generating new tasks...\n');
        for j = 1:num_tasks
            tasks(j).pos = [rand*env_size, rand*env_size, rand*15 + 5];
            tasks(j).assigned = false;
            tasks(j).completed = false;
        end
    end
end

hold off;
fprintf('Simulation Stopped.\n');
