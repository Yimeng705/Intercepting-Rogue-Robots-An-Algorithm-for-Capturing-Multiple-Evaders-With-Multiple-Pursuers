% 微分博弈参数设置
alpha = 0.8;  % 预测时间权重系数
max_prediction_time = 2.0;  % 最大预测时间（秒）

for i = 1:length(pursuers)
    % 1. 计算所有逃避者的价值（考虑距离和博弈收益）
    evader_values = zeros(1, n_e_alive);
    for j = 1:n_e_alive
        evader_pos = pos_evaders(j, :);
        dist = norm(pursuers{i}.position - evader_pos);
        
        % 微分博弈核心：计算收益函数
        % 考虑追逃相对位置、速度方向、障碍物等因素
        [predicted_pos, prediction_time] = predictEvaderMovement(...
            evaders{evaders_alive_index(j)}, ...
            pursuers{i}.position, ...
            max_prediction_time);
        
        % 价值计算 = 距离因素 + 博弈收益
        pursuit_gain = calculatePursuitGain(...
            pursuers{i}.position, ...
            evader_pos, ...
            predicted_pos, ...
            pursuers{i}.velocity);
        
        evader_values(j) = alpha/(dist + 0.1) + (1-alpha)*pursuit_gain;
    end

    % 2. 选择最佳目标（最高价值逃避者）
    [~, bestEvaderIdx] = max(evader_values);
    selected_evader = evaders{evaders_alive_index(bestEvaderIdx)};
    
    % 3. 微分博弈驱动的最优控制
    if adjacencyMatrix(i + n_e_alive, bestEvaderIdx)
        % 当逃避者是Voronoi邻居时，采用最优追捕策略
        optimal_velocity = differentialPursuitStrategy(...
            pursuers{i}, ...
            selected_evader);
        
        pursuers{i}.velocity = optimal_velocity;
        pursuers{i}.targetIsAdjacent = true;
    else
        % 非邻居时采用预测性拦截策略
        [predicted_pos, ~] = predictEvaderMovement(...
            selected_evader, ...
            pursuers{i}.position, ...
            max_prediction_time);
        
        pursuers{i}.target.position = predicted_pos;
        pursuers{i}.targetIsAdjacent = false;
        pursuers{i} = pursuers{i}.calculateVelocity();
    end
    
    % 4. 更新追捕者状态
    pursuers{i}.target = selected_evader;
    pursuers{i}.last_decision_time = t;  % 记录决策时间
end

% ======== 微分博弈辅助函数 ========

% 预测逃避者移动（考虑最优逃避策略）
function [predicted_pos, prediction_time] = predictEvaderMovement(...
    evader, pursuer_pos, max_time)
    % 基本参数
    evasion_dir = evader.position - pursuer_pos;
    if norm(evasion_dir) > 0
        evasion_dir = evasion_dir/norm(evasion_dir);
    else
        evasion_dir = [1, 0];  % 默认向右逃逸
    end
    
    % 计算最优预测时间（基于相对距离和速度）
    rel_speed = norm(evader.max_speed) + norm(evader.velocity);
    dist = norm(evader.position - pursuer_pos);
    prediction_time = min(max_time, dist/(rel_speed + 0.1));
    
    % 预测位置 = 当前位置 + 最优逃避方向×速度×时间
    optimal_evasion_vector = evasion_dir * evader.max_speed * prediction_time;
    predicted_pos = evader.position + optimal_evasion_vector;
end

% 计算追捕收益函数
function gain = calculatePursuitGain(pursuer_pos, evader_pos, predicted_pos, pursuer_vel)
    % 当前相对位置向量
    current_vec = evader_pos - pursuer_pos;
    
    % 预测相对位置向量
    predicted_vec = predicted_pos - pursuer_pos;
    
    % 速度对齐因子（1表示完美对齐）
    velocity_alignment = dot(pursuer_vel, predicted_vec) / ...
        (norm(pursuer_vel)*norm(predicted_vec) + 0.01);
    
    % 距离缩减因子
    dist_reduction = norm(current_vec) - norm(predicted_vec);
    
    % 综合收益
    gain = 0.7*velocity_alignment + 0.3*dist_reduction/(norm(current_vec)+0.1);
end

% 微分追捕策略（HJI方程简化实现）
function optimal_vel = differentialPursuitStrategy(pursuer, evader)
    % 状态参数
    rel_pos = evader.position - pursuer.position;
    rel_vel = evader.velocity - pursuer.velocity;
    
    % 简化的Hamilton-Jacobi-Isaacs控制
    K_p = 1.2;  % 位置增益
    K_v = 0.8;  % 速度增益
    
    % 最优控制输入 (简化模型)
    optimal_vel = K_p*rel_pos + K_v*rel_vel;
    
    % 速度限制
    max_speed = pursuer.max_speed;
    if norm(optimal_vel) > max_speed
        optimal_vel = optimal_vel/norm(optimal_vel)*max_speed;
    end
end