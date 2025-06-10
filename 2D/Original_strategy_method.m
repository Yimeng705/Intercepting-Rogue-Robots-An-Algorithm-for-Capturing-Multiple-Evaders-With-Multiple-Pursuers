function [captureTime, firstCaptureTime] = Original_strategy_method(...
    pursuerPos, evaderPos, timestep, timeend, xbound, ybound)
% ORIGINAL_STRATEGY 实现原始追捕策略
%   输入参数:
%       pursuerPos - 追捕者初始位置 (n_p x 2 矩阵)
%       evaderPos  - 逃避者初始位置 (n_e x 2 矩阵)
%       timestep   - 仿真时间步长
%       timeend    - 最大仿真时间
%       xbound     - X边界 [min, max]
%       ybound     - Y边界 [min, max]
%
%   输出参数:
%       captureTime     - 总捕获时间
%       firstCaptureTime - 首次捕获时间

% 创建边界多边形
bbox = [xbound(1), ybound(1); xbound(2), ybound(1); ...
        xbound(2), ybound(2); xbound(1), ybound(2)];
bound = Polyhedron(bbox);

% 初始化变量
n_p = size(pursuerPos, 1); % 追捕者数量
n_e = size(evaderPos, 1);  % 逃避者数量
captureTime = timeend;      % 默认设置为最大值
firstCaptureTime = inf;     % 初始化为无穷大

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

% 主循环
for t = 0:timestep:timeend
    n_e_alive = 0;          % 存活的逃避者数量
    evaders_alive_index = [];% 存活的逃避者索引
    pos_evaders = [];        % 存活的逃避者位置
    
    % 1. 检查逃避者状态并收集存活者信息
    for i = 1:length(evaders)
        if evaders{i}.isDead
            continue;
        else
            evaders{i} = evaders{i}.checkIfAlive(pursuers);
            
            if ~evaders{i}.isDead
                n_e_alive = n_e_alive + 1;
                evaders_alive_index = [evaders_alive_index, i];
                pos_evaders = [pos_evaders; evaders{i}.position];
            end
        end
    end
    
    % 检测首次捕获
    if isinf(firstCaptureTime) && n_e_alive < n_e
        firstCaptureTime = t;
    end
    
    % 获取追捕者位置
    pos_pursuers = zeros(n_p, 2);
    for i = 1:length(pursuers)
        pos_pursuers(i, :) = pursuers{i}.position;
    end
    
    % 检查是否所有逃避者都被捕获
    if n_e_alive == 0
        captureTime = t;
        break;
    end
    
    % 2. 计算Voronoi图
    pos_all = [pos_evaders; pos_pursuers];
    [~, p] = mpt_voronoi(pos_all', 'bound', bound);
    
    % 分配Voronoi单元
    for i = 1:length(p)
        if i <= n_e_alive
            evaders{evaders_alive_index(i)} = ...
                evaders{evaders_alive_index(i)}.setVoronoiCell(p(i));
        else
            pursuers{i - n_e_alive} = ...
                pursuers{i - n_e_alive}.setVoronoiCell(p(i));
        end
    end
    
    % 3. 获取邻接矩阵
    adjacencyMatrix = getVoronoiAdjacency(p);
    
    % 4. 更新追捕者行为
    for i = 1:length(pursuers)
        % 寻找最近的逃避者
        dists = pdist2(pursuers{i}.position, pos_evaders);
        [~, nearestEvaderIdx] = min(dists);
        
        % 设置目标逃避者
        pursuers{i}.target = evaders{evaders_alive_index(nearestEvaderIdx)};
        
        % 检查是否相邻
        if adjacencyMatrix(i + n_e_alive, nearestEvaderIdx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        % 计算速度
        pursuers{i} = pursuers{i}.calculateVelocity();
    end
    
    % 5. 更新逃避者行为
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            evaders{i} = evaders{i}.calculateVelocity();
        end
    end
    
    % 6. 移动所有智能体
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            evaders{i} = evaders{i}.move(timestep);
        end
    end
    for i = 1:length(pursuers)
        pursuers{i} = pursuers{i}.move(timestep);
    end
end

% 如果所有逃避者未被捕获，使用最大时间
if captureTime == timeend
    captureTime = timeend;
end
end