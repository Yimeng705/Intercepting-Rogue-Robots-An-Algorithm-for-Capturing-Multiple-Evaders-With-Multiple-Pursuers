clear;
clc;
close all;

numRuns = 10;  % 运行次数
timestep = 0.1; % 仿真时间步长
timeend = 200;
xbound = [-10,10];
ybound = [-10,10];
bbox = [xbound(1), ybound(1);xbound(2), ybound(1);xbound(2), ybound(2);xbound(1), ybound(2)];

%xlim(xbound);
%ylim(ybound);

% 初始化结果存储
endTimes = zeros(numRuns, 1);  % 每次运行的结束时间
captureTimes = zeros(numRuns, 1); % 每次捕捉完成的时间
firstCaptureTimes = zeros(numRuns, 1);  % 初始化首次捕获时间

for runIdx = 1:numRuns
    firstCaptureTime = inf;
    fprintf('开始运行 #%d/%d\n', runIdx, numRuns);
    tic;  % 开始计时实际运行时间
    
    n_p = 4; % number of pursuers
    n_e = 10; % number of evaders
    
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
    
        count = 1;
        for i = 1:length(p)
            if i <= n_e_alive
                evaders{evaders_alive_index(i)} = evaders{evaders_alive_index(i)}.setVoronoiCell(p(i));
            else
                pursuers{i-n_e_alive} = pursuers{i-n_e_alive}.setVoronoiCell(p(i));
            end
        end
    
        adjacencyMatrix = getVoronoiAdjacency(p);
        
        for i = 1:length(pursuers)
            % find the nearest evader
            dists = pdist2(pursuers{i}.position, pos_evaders);
            [~, nearestEvaderIdx] = min(dists);
            
            % 2. determine nearest evader from Voronoi neighbors
            for j = 1:n_e_alive
                
            end
    
            pursuers{i}.target = evaders{evaders_alive_index(nearestEvaderIdx)};
            if adjacencyMatrix(i + n_e_alive, nearestEvaderIdx)
                pursuers{i}.targetIsAdjacent = true;
            else
                pursuers{i}.targetIsAdjacent = false;
            end
            
            % 更新pursuers{i}的速度
            pursuers{i} = pursuers{i}.calculateVelocity();
            
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
    
    end
    
    % 记录本次运行结果
    endTimes(runIdx) = toc; 
    captureTimes(runIdx) = captureTime;
    firstCaptureTimes(runIdx) = firstCaptureTime; 
    fprintf('运行 #%d 完成: 捕捉时间 = %.2f s, 实际耗时 = %.2f s\n', ...
            runIdx, captureTime, endTimes(runIdx));
    fprintf('首次捕获时间: %.2f s\n', firstCaptureTime);
end

% 计算统计结果
avgCaptureTime = mean(captureTimes);
stdCaptureTime = std(captureTimes);
avgfirstCaptureTime = mean(firstCaptureTimes);
stdfirstCaptureTime = std(firstCaptureTimes);  % 新增首次捕获时间标准差

% 绘制捕捉时间结果
figure;
subplot(2,1,1);  % 分两个子图[3](@ref)
plot(1:numRuns, captureTimes, 'o-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');  % 增强线条和标记[1](@ref)
title('每次仿真的总捕捉时间', 'FontSize', 12, 'FontWeight', 'bold');  % 增强标题[5](@ref)
xlabel('运行次数');
ylabel('时间 (s)');
grid on;
hold on;
yline(avgCaptureTime, '--r', 'LineWidth', 1.5, 'Label', ...
    sprintf('平均时间: %.2f±%.2f s', avgCaptureTime, stdCaptureTime));  % 添加标准差[6](@ref)
legend('单次捕捉时间', '平均时间', 'Location', 'best');

% 新增首次捕获时间可视化
subplot(2,1,2);
plot(1:numRuns, firstCaptureTimes, 's-', 'Color', [0.9, 0.4, 0], ...
    'MarkerFaceColor', [0.9, 0.6, 0]);  % 橙色系区分[1](@ref)
title('每次仿真的首次捕获时间', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('运行次数');
ylabel('时间 (s)');
grid on;
hold on;
yline(avgfirstCaptureTime, '--', 'Color', [0.5, 0, 0.5], 'LineWidth', 1.5, ...
    'Label', sprintf('平均时间: %.2f±%.2f s', avgfirstCaptureTime, stdfirstCaptureTime));  % 紫色区分[1](@ref)
legend('首次捕获时间', '平均时间', 'Location', 'best');

% 优化图形显示
set(gcf, 'Position', [100, 100, 800, 600]);  % 设置图形大小[3](@ref)
set(gca, 'LooseInset', [0,0,0,0]);  % 去除白边[1](@ref)

% 显示统计结果
fprintf('\n===== 统计结果 =====\n');
fprintf('总捕捉时间 - 平均值: %.4f ± %.4f 秒\n', avgCaptureTime, stdCaptureTime);
fprintf('        最短时间: %.2f 秒 | 最长时间: %.2f 秒\n', min(captureTimes), max(captureTimes));

fprintf('\n首次捕获时间 - 平均值: %.4f ± %.4f 秒\n', avgfirstCaptureTime, stdfirstCaptureTime);
fprintf('        最短时间: %.2f 秒 | 最长时间: %.2f 秒\n', min(firstCaptureTimes), max(firstCaptureTimes));  % 新增极值统计

% 计算捕获效率提升比
captureEfficiency = (captureTimes - firstCaptureTimes) ./ captureTimes * 100;
fprintf('\n捕获效率提升比: %.1f%% (首次捕获后平均加速)\n', mean(captureEfficiency));