function [captureTime, firstCaptureTime] = Differential_game_strategy_method_3D(...
    pursuerPos, evaderPos, timestep, timeend, xbound, ybound, zbound)
% DIFFERENTIAL_GAME_STRATEGY_3D 实现3D空间中的多智能体追捕微分博弈策略
% 输入:
%   pursuerPos - 追捕者初始位置 [Np x 3] 矩阵
%   evaderPos  - 逃避者初始位置 [Ne x 3] 矩阵
%   timestep   - 仿真时间步长 (默认0.1)
%   timeend    - 最大仿真时间 (默认200)
%   xbound     - x轴边界 [xmin, xmax] (默认[-10,10])
%   ybound     - y轴边界 [ymin, ymax] (默认[-10,10])
%   zbound     - z轴边界 [zmin, zmax] (默认[-10,10])

% 创建边界立方体
bbox = [xbound(1), ybound(1), zbound(1); 
        xbound(2), ybound(1), zbound(1);
        xbound(2), ybound(2), zbound(1);
        xbound(1), ybound(2), zbound(1);
        xbound(1), ybound(1), zbound(2);
        xbound(2), ybound(1), zbound(2);
        xbound(2), ybound(2), zbound(2);
        xbound(1), ybound(2), zbound(2)];

% 初始化变量
n_p = size(pursuerPos, 1); % 追捕者数量
n_e = size(evaderPos, 1);  % 逃避者数量
captureTime = timeend;      % 默认设置为最大值
firstCaptureTime = inf;     % 初始化为无穷大

% 微分博弈参数设置
alpha = 0.8;  % 预测时间权重系数
max_prediction_time = 2.0;  % 最大预测时间（秒）

% 创建追捕者和逃避者对象
pursuers = cell(1, n_p);
evaders = cell(1, n_e);

% initialize the pursuers
for i=1:n_p
    pos = rand(1,3);
    pos(1) = pos(1) * (xbound(2) - xbound(1)) + xbound(1);
    pos(2) = pos(2) * (ybound(2) - ybound(1)) + ybound(1);
    pos(3) = pos(3) * (zbound(2) - zbound(1)) + zbound(1);
    pursuers{i} = Pursuer3D(pos);
end

% initialize the evaders
for i=1:n_e
    pos = rand(1,3);
    pos(1) = pos(1) * (xbound(2) - xbound(1)) + xbound(1);
    pos(2) = pos(2) * (ybound(2) - ybound(1)) + ybound(1);
    pos(3) = pos(3) * (zbound(2) - zbound(1)) + zbound(1);
    evaders{i} = Evader3D(pos);
end

bound = Polyhedron(bbox);   % 3D边界
captureTime = timeend; % 默认设置为最大值

pos_pursuers = zeros(n_p, 3);
for t=0:timestep:timeend
    n_e_alive = 0;      % 记录当前存活的evaders数量
    evaders_alive_index = [];   % 记录存活的evader在evaders中的索引
    pos_evaders = [];
    target_assignment = zeros(1, n_e_alive); % 记录每个逃避者被几个追捕者选择

    % 1. 更新逃避者状态
    for i = 1:length(evaders)
        if evaders{i}.isDead
            continue;
        else
            wasDead = evaders{i}.isDead;  % 记录检查前的状态
            evaders{i} = evaders{i}.checkIfAlive(pursuers);
            
            if ~evaders{i}.isDead
                n_e_alive = n_e_alive + 1;
                evaders_alive_index = [evaders_alive_index, i];
                pos_evaders = [pos_evaders; evaders{i}.position];   % 存活的evaders位置
            end
        end
    end
    
    % 检测首次捕获
    if isinf(firstCaptureTime) && n_e_alive < n_e
        firstCaptureTime = t;  % 记录当前仿真时间
    end

    for i = 1:length(pursuers)
        pos_pursuers(i,:) = pursuers{i}.position;
    end

    % 所有逃避者都被捕获
    if n_e_alive == 0
        captureTime = t;
        break;
    end
    
    % 2. 计算3D Voronoi图
    pos_all = [pos_evaders; pos_pursuers];
    [v, p] = voronoiDiagram(pos_all);  % MATLAB内置3D Voronoi函数

    % 3. 将Voronoi单元分配给智能体
    for i = 1:length(p)
        if i <= n_e_alive
            evaders{evaders_alive_index(i)}.voronoi_cell = p{i};
        else
            pursuers{i-n_e_alive}.voronoi_cell = p{i};
        end
    end

    % 4. 计算Voronoi邻接矩阵
    adjacencyMatrix = getVoronoiAdjacency3D(p);
    
    % 5. 追捕者决策
    for i = 1:length(pursuers)
        % 微分博弈目标选择
        evader_values = zeros(1, n_e_alive);

        for j = 1:n_e_alive
            evader_pos = pos_evaders(j, :);
            dist = norm(pursuers{i}.position - evader_pos);
            
            % 预测逃避者最优移动轨迹（3D）
            [predicted_pos, prediction_time] = predictEvaderMovement3D(...
                evaders{evaders_alive_index(j)}, ...
                pursuers{i}.position, ...
                max_prediction_time, ...
                [xbound; ybound; zbound]);
            
            % 计算微分博弈收益
            pursuit_gain = calculatePursuitGain3D(...
                pursuers{i}.position, ...
                evader_pos, ...
                predicted_pos, ...
                pursuers{i}.velocity);
            
            % 价值 = 距离因素 + 博弈收益
            evader_values(j) = alpha/(dist + 0.1) + (1-alpha)*pursuit_gain;
        end
        
        % 选择最佳目标（最高价值逃避者）
        [~, bestEvaderIdx] = max(evader_values);
        selected_evader = evaders{evaders_alive_index(bestEvaderIdx)};
       
        % 微分博弈驱动的最优控制
        if adjacencyMatrix(i + n_e_alive, bestEvaderIdx) % Voronoi邻居判定
            % 微分追捕策略（3D）
            optimal_velocity = differentialPursuitStrategy3D(...
                pursuers{i}, selected_evader);
            pursuers{i}.velocity = optimal_velocity;
            pursuers{i}.targetIsAdjacent = true;
        else
            % 非邻居时采用预测性拦截
            [predicted_pos, ~] = predictEvaderMovement3D(...
                selected_evader, pursuers{i}.position, max_prediction_time, ...
                [xbound; ybound; zbound]);
            
            pursuers{i}.target.position = predicted_pos;
            pursuers{i}.targetIsAdjacent = false;
            pursuers{i} = pursuers{i}.calculateVelocity(); % 保持原速度计算
        end
        
        % 应急策略：长时间未捕获目标
        if (t - pursuers{i}.last_capture_progress) > 15
            dist = norm(pursuers{i}.position - selected_evader.position);
            if dist > pursuers{i}.max_speed * 5
                % 长距离预测拦截
                [predicted_pos, ~] = predictEvaderMovement3D(...
                    selected_evader, pursuers{i}.position, max_prediction_time*2, ...
                    [xbound; ybound; zbound]);
                intercept_dir = predicted_pos - pursuers{i}.position;
                optimal_vel = pursuers{i}.max_speed * (intercept_dir/norm(intercept_dir));
            else
                % 直线冲刺
                charge_dir = selected_evader.position - pursuers{i}.position;
                optimal_vel = pursuers{i}.max_speed * 1.2 * (charge_dir/norm(charge_dir));
            end
            pursuers{i}.velocity = optimal_vel;
        end

        % 更新进展时间
        if norm(pursuers{i}.position - selected_evader.position) < 0.5
            pursuers{i}.last_capture_progress = t;
        end

        % 更新状态
        pursuers{i}.target = selected_evader;
        pursuers{i}.last_decision_time = t;
    end

    % 6. 逃避者移动决策
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            evaders{i} = evaders{i}.calculateVelocity3D();
        end
    end

    % 7. 更新所有智能体位置
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            evaders{i} = evaders{i}.move(timestep, [xbound; ybound; zbound]);
        end
    end
    for i = 1:length(pursuers)
        pursuers{i} = pursuers{i}.move(timestep, [xbound; ybound; zbound]);
    end

    % 8. 检查逃避者是否被捕获
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            evaders{i} = evaders{i}.checkIfAlive(pursuers);
        end
    end
end

% 如果所有逃避者未被捕获，使用最大时间
if captureTime == timeend
    captureTime = timeend;
end
end

%% 3D辅助函数
function [predicted_pos, prediction_time] = predictEvaderMovement3D(...
    evader, pursuer_pos, max_time, bounds)
    % 计算Voronoi单元的质心（3D）
    if ~isempty(evader.voronoi_cell)
        centroid = mean(evader.voronoi_cell, 1);
    else
        centroid = evader.position; % 备用方案
    end
    
    centroid_dir = centroid - evader.position;
    if norm(centroid_dir) > 0
        evasion_dir = centroid_dir / norm(centroid_dir);
    else
        % 随机逃逸方向
        evasion_dir = rand(1,3)*2 - 1;
        evasion_dir = evasion_dir / norm(evasion_dir);
    end
    
    % 自适应预测时间
    rel_speed = norm(evader.velocity) + evader.max_speed;
    dist = norm(evader.position - pursuer_pos);
    prediction_time = min(max_time, dist/(rel_speed + 0.1));
    
    % 预测位置 = 当前位置 + 方向×速度×时间
    predicted_pos = evader.position + evasion_dir * evader.max_speed * prediction_time;

    % 边界约束处理
    predicted_pos = constrainPosition3D(predicted_pos, bounds);
end

function gain = calculatePursuitGain3D(pursuer_pos, evader_pos, predicted_pos, pursuer_vel)
    % 当前与预测的相对位置向量
    current_vec = evader_pos - pursuer_pos;
    predicted_vec = predicted_pos - pursuer_pos;
    
    % 速度对齐因子
    if norm(pursuer_vel) > 0
        velocity_alignment = dot(pursuer_vel, predicted_vec) / ...
            (norm(pursuer_vel)*norm(predicted_vec));
    else
        velocity_alignment = 1; % 静止时视为最佳对齐
    end
    
    % 距离缩减因子
    dist_reduction = norm(current_vec) - norm(predicted_vec);
    
    % 综合收益（可调整权重）
    gain = 0.6*velocity_alignment + 0.4*dist_reduction/(norm(current_vec)+0.1);
end

function optimal_vel = differentialPursuitStrategy3D(pursuer, evader)
    % 参数设置
    K_p = 1.5;
    K_v = 1.0;
    intercept_gain = 0.6;
    
    % 计算最优拦截点 (3D版)
    rel_pos = evader.position - pursuer.position;
    rel_vel = evader.velocity - pursuer.velocity;
    closing_speed = -dot(rel_pos, rel_vel)/(norm(rel_pos)+1e-5);
    intercept_time = norm(rel_pos)/(closing_speed + 1e-5);
    intercept_point = evader.position + evader.velocity * intercept_time;
    
    % 混合策略：70%拦截点 + 30%直接追踪
    target_vec = intercept_gain*(intercept_point - pursuer.position) + ...
                 (1-intercept_gain)*rel_pos;
    
    % 速度控制
    optimal_vel = K_p * target_vec + K_v * rel_vel;
    
    % 速度约束
    max_speed = pursuer.max_speed;
    if norm(optimal_vel) > max_speed
        optimal_vel = optimal_vel * (max_speed/norm(optimal_vel));
    end
end

function constrained_pos = constrainPosition3D(pos, bounds)
    % 提取边界
    x_min = bounds(1,1); x_max = bounds(1,2);
    y_min = bounds(2,1); y_max = bounds(2,2);
    z_min = bounds(3,1); z_max = bounds(3,2);
    
    % 应用边界约束
    constrained_x = min(max(pos(1), x_min), x_max);
    constrained_y = min(max(pos(2), y_min), y_max);
    constrained_z = min(max(pos(3), z_min), z_max);
    
    constrained_pos = [constrained_x, constrained_y, constrained_z];
end

function adjacencyMatrix = getVoronoiAdjacency3D(cells)
    % 计算3D Voronoi图的邻接矩阵
    n = length(cells);
    adjacencyMatrix = zeros(n);
    
    for i = 1:n
        for j = i+1:n
            % 检查两个单元是否共享一个面
            if shareFace(cells{i}, cells{j})
                adjacencyMatrix(i,j) = 1;
                adjacencyMatrix(j,i) = 1;
            end
        end
    end
end

function result = shareFace(poly1, poly2)
    % 检查两个3D多边形是否共享一个面
    result = false;
    if isempty(poly1) || isempty(poly2)
        return;
    end
    
    % 计算两个多面体的交集
    P1 = Polyhedron(poly1);
    P2 = Polyhedron(poly2);
    intersect = P1.intersect(P2);
    
    % 如果交集是一个二维多边形（面），则返回true
    if intersect.Dim == 2 && intersect.Volume > 1e-5
        result = true;
    end
end

%% 3D智能体类
classdef Pursuer3D < handle
    properties
        position
        velocity
        max_speed = 2.0
        target
        voronoi_cell
        targetIsAdjacent = false
        last_decision_time = 0
        last_capture_progress = 0
    end
    
    methods
        function obj = Pursuer3D(pos)
            obj.position = pos;
            obj.velocity = [0,0,0];
        end
        
        function obj = setVoronoiCell(obj, cell)
            obj.voronoi_cell = cell;
        end
        
        function obj = calculateVelocity(obj)
            if ~isempty(obj.target)
                dir = obj.target.position - obj.position;
                if norm(dir) > 0
                    obj.velocity = obj.max_speed * (dir/norm(dir));
                end
            end
        end
        
        function obj = move(obj, dt, bounds)
            obj.position = obj.position + obj.velocity * dt;
            obj.position = constrainPosition3D(obj.position, bounds);
        end
    end
end

classdef Evader3D < handle
    properties
        position
        velocity
        max_speed = 1.5
        voronoi_cell
        isDead = false
    end
    
    methods
        function obj = Evader3D(pos)
            obj.position = pos;
            obj.velocity = [0,0,0];
        end
        
        function obj = setVoronoiCell(obj, cell)
            obj.voronoi_cell = cell;
        end
        
        function obj = calculateVelocity3D(obj)
            if ~isempty(obj.voronoi_cell)
                centroid = mean(obj.voronoi_cell, 1);
                dir = centroid - obj.position;
                if norm(dir) > 0
                    obj.velocity = obj.max_speed * (dir/norm(dir));
                else
                    % 随机逃逸方向
                    obj.velocity = obj.max_speed * (rand(1,3)-0.5);
                    obj.velocity = obj.velocity / norm(obj.velocity);
                end
            end
        end
        
        function obj = move(obj, dt, bounds)
            obj.position = obj.position + obj.velocity * dt;
            obj.position = constrainPosition3D(obj.position, bounds);
        end
        
        function obj = checkIfAlive(obj, pursuers)
            for j = 1:length(pursuers)
                if norm(obj.position - pursuers{j}.position) < 0.5
                    obj.isDead = true;
                    return;
                end
            end
        end
    end
end