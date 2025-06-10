function [captureTime, firstCaptureTime] = Differential_game_strategy_method(...
    pursuerPos, evaderPos, timestep, timeend, xbound, ybound)
% DIFFERENTIAL_GAME_STRATEGY 实现多智能体追捕的微分博弈策略
% 输入:
%   pursuerPos - 追捕者初始位置 [Np x 2] 矩阵
%   evaderPos  - 逃避者初始位置 [Ne x 2] 矩阵
%   timestep   - 仿真时间步长 (默认0.1)
%   timeend    - 最大仿真时间 (默认200)
%   xbound     - x轴边界 [xmin, xmax] (默认[-10,10])
%   ybound     - y轴边界 [ymin, ymax] (默认[-10,10])
% 输出:
%   captureTime    - 所有逃避者被捕获的总时间
%   firstCaptureTime - 首次捕获发生的时间

% 创建边界多边形
bbox = [xbound(1), ybound(1); xbound(2), ybound(1); ...
        xbound(2), ybound(2); xbound(1), ybound(2)];

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
    pos = rand(1,2);
    pos(1,1) = pos(1,1) * (xbound(2) - xbound(1)) + xbound(1);
    pos(1,2) = pos(1,2) * (ybound(2) - ybound(1)) + ybound(1);
    pursuers{i} = Pursuer(pos);
end

% initialize the evaders
for i=1:n_e
    pos = rand(1,2);
    pos(1,1) = pos(1,1) * (xbound(2) - xbound(1)) + xbound(1);
    pos(1,2) = pos(1,2) * (ybound(2) - ybound(1)) + ybound(1);
    evaders{i} = Evader(pos);
end

bound = Polyhedron(bbox);   % 边界
captureTime = timeend; % 默认设置为最大值
% TODO: 主循环
pos_pursuers = zeros(n_p, 2);
for t=0:timestep:timeend
    n_e_alive = 0;      % 记录当前存活的evaders数量
    evaders_alive_index = [];   % 记录存活的evader在evaders中的索引
    pos_evaders = [];
    target_assignment = zeros(1, n_e_alive); % 记录每个逃避者被几个追捕者选择

    % 1. calculate Voronoi tesselation with all agents
    for i = 1:length(evaders)
        if evaders{i}.isDead
            continue;
        else
            wasDead = evaders{i}.isDead;  % 记录检查前的状态
            evaders{i} = evaders{i}.checkIfAlive(pursuers);

            n_e_alive = n_e_alive + 1;
            
            evaders_alive_index = [evaders_alive_index, i];
            pos_evaders = [pos_evaders; evaders{i}.position];   % 存活的evaders位置加入pos_evaders
        end
    end
    
    % 检测首次捕获：从存活变为死亡且未记录过首次捕获
    if isinf(firstCaptureTime) && n_e_alive == n_e - 1
        firstCaptureTime = t;  % 记录当前仿真时间
    end

    for i = 1:length(pursuers)
        pos_pursuers(i,:) = pursuers{i}.position;
    end

    if n_e_alive ==0
        captureTime = t;
        break;
    end
    

    % get voronoi diagram by using function voronoi()
    pos_all = [pos_evaders; pos_pursuers];
    [v,p] = mpt_voronoi(pos_all', 'bound', bound);  % 取出所有agents的位置用于计算voronoi图

    for i = 1:length(p)
        if i <= n_e_alive
            evaders{evaders_alive_index(i)} = evaders{evaders_alive_index(i)}.setVoronoiCell(p(i));
        else
            pursuers{i-n_e_alive} = pursuers{i-n_e_alive}.setVoronoiCell(p(i));
        end
    end

    adjacencyMatrix = getVoronoiAdjacency(p);
    
    for i = 1:length(pursuers)
        
        % 1. 微分博弈目标选择（替换最近邻选择）
        evader_values = zeros(1, n_e_alive);

        for j = 1:n_e_alive
            evader_pos = pos_evaders(j, :);
            dist = norm(pursuers{i}.position - evader_pos);
            
            % 预测逃避者最优移动轨迹（关键改进）
            [predicted_pos, prediction_time] = predictEvaderMovement(...
                evaders{evaders_alive_index(j)}, ...
                pursuers{i}.position, ...
                max_prediction_time);
            
            % 计算微分博弈收益（核心创新）
            pursuit_gain = calculatePursuitGain(...
                pursuers{i}.position, ...
                evader_pos, ...
                predicted_pos, ...
                pursuers{i}.velocity);
            
            % 价值 = 距离因素 + 博弈收益（α控制权重）
            evader_values(j) = alpha/(dist + 0.1) + (1-alpha)*pursuit_gain;
        end
        
        % 2. 选择最佳目标（最高价值逃避者）
        [~, bestEvaderIdx] = max(evader_values);
        selected_evader = evaders{evaders_alive_index(bestEvaderIdx)};
       
        % 在控制策略中应用凸优化
        % 3. 微分博弈驱动的最优控制
        if adjacencyMatrix(i + n_e_alive, bestEvaderIdx) % Voronoi邻居判定
            % 微分追捕策略（HJI方程简化实现）
            optimal_velocity = differentialPursuitStrategy(...
                pursuers{i}, selected_evader);
            pursuers{i}.velocity = optimal_velocity;
            pursuers{i}.targetIsAdjacent = true;
        else
            % 非邻居时采用预测性拦截
            [predicted_pos, ~] = predictEvaderMovement(...
                selected_evader, pursuers{i}.position, max_prediction_time);
            
            pursuers{i}.target.position = predicted_pos;
            pursuers{i}.targetIsAdjacent = false;
            pursuers{i} = pursuers{i}.calculateVelocity(); % 保持原速度计算
        end
        
        % 在追捕者循环内添加应急策略
        if (t - pursuers{i}.prediction_time) > 15  % 15秒无进展
            % 切换到激进模式
            dist = norm(pursuers{i}.position - selected_evader.position);
            if dist > pursuers{i}.max_speed * 5
                % 启用预测拦截增强
                [predicted_pos, ~] = predictEvaderMovement(...
                    selected_evader, pursuers{i}.position, max_prediction_time*2);
                intercept_dir = predicted_pos - pursuers{i}.position;
                optimal_vel = pursuers{i}.max_speed * (intercept_dir/norm(intercept_dir));
            else
                % 最后阶段直线冲刺
                charge_dir = selected_evader.position - pursuers{i}.position;
                optimal_vel = pursuers{i}.max_speed * 1.2 * (charge_dir/norm(charge_dir));
            end
        end

         % 在checkIfAlive后更新进展时间
        if any(vecnorm(pos_pursuers - selected_evader.position, 2, 2) < 0.5)
            pursuers{i}.last_capture_progress = t;
        end


        % 4. 更新状态
        pursuers{i}.target = selected_evader;
        pursuers{i}.last_decision_time = t;
    end


    % 5. calculate u_e^\kappa
    for i = 1:length(evaders)
        evaders{i} = evaders{i}.calculateVelocity();
    end

    % 6. move the robots
    for i = 1:length(evaders)
        evaders{i} = evaders{i}.move(timestep);
    end
    for i = 1:length(pursuers)
        pursuers{i} = pursuers{i}.move(timestep);
    end

    % check if evader is dead
    for i = 1:length(evaders)
        if evaders{i}.isDead
            continue;
        else
            evaders{i} = evaders{i}.checkIfAlive(pursuers);
        end
    end

    % 如果所有逃避者未被捕获，使用最大时间
    if captureTime == timeend
        captureTime = timeend;
    end
end
end

function [predicted_pos, prediction_time] = predictEvaderMovement(...
    evader, pursuer_pos, max_time)
    centroid = mean(evader.voronoi_cell.V, 1);
    centroid_dir = centroid - evader.position;
    if norm(centroid_dir) > 0
        evasion_dir = centroid_dir / norm(centroid_dir);
    else
        evasion_dir = [0, 0];
    end
    
    % 自适应预测时间
    rel_speed = norm(evader.getVelocity()) + norm(evader.max_speed);
    dist = norm(evader.position - pursuer_pos);
    prediction_time = min(max_time, dist/(rel_speed + 0.1));
    
    % 预测位置 = 当前位置 + 方向×速度×时间
    predicted_pos = evader.position + evasion_dir * evader.max_speed * prediction_time;

    xbound = [-10,10];
    ybound = [-10,10];
    bbox = [xbound(1), ybound(1);xbound(2), ybound(1);xbound(2), ybound(2);xbound(1), ybound(2)];
    global_boundary = bbox;
    % 边界约束处理
    predicted_pos = constrainPosition(predicted_pos, global_boundary);
end

function gain = calculatePursuitGain(pursuer_pos, evader_pos, predicted_pos, pursuer_vel)
    % 当前与预测的相对位置向量
    current_vec = evader_pos - pursuer_pos;
    predicted_vec = predicted_pos - pursuer_pos;
    
    % 速度对齐因子（1表示完美朝向目标）
    if norm(pursuer_vel) > 0
        velocity_alignment = dot(pursuer_vel, predicted_vec) / ...
            (norm(pursuer_vel)*norm(predicted_vec));
    else
        velocity_alignment = 1; % 静止时视为最佳对齐
    end
    
    % 距离缩减因子（预测距离变化）
    dist_reduction = norm(current_vec - predicted_vec);
    
    % 综合收益
    gain = 0.7*velocity_alignment + 0.3*dist_reduction/(norm(current_vec)+0.1);
end

function optimal_vel = differentialPursuitStrategy(pursuer, evader)
    % 参数设置
    K_p = 1.5;
    K_v = 1.0;
    intercept_gain = 0.6;
    
    % 计算最优拦截点 (ISAACS方程简化)
    rel_pos = evader.position - pursuer.position;
    rel_vel = evader.velocity - pursuer.velocity;
    closing_speed = -dot(rel_pos, rel_vel)/norm(rel_pos);
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

function constrained_pos = constrainPosition(pos, boundary)
    % 提取边界极值
    x_min = min(boundary(:,1));
    x_max = max(boundary(:,1));
    y_min = min(boundary(:,2));
    y_max = max(boundary(:,2));
    
    % 应用边界约束
    constrained_x = min(max(pos(1), x_min), x_max);
    constrained_y = min(max(pos(2), y_min), y_max);
    
    constrained_pos = [constrained_x, constrained_y];
end



