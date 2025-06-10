% strategy_comparison.m
clear; clc; close all;

% 确保结果可复现
rng(42);  

% 参数设置
numRuns = 30;   % 运行次数
timestep = 0.1;     % 时间步长
timeend = 200;      % 最大仿真时间
xbound = [-10,10];  % X边界
ybound = [-10,10];  % Y边界
n_p = 4;            % 追捕者数量
n_e = 10;           % 逃避者数量

% 存储所有初始位置（保证两种策略使用相同的初始条件）
initialPositions = cell(numRuns, 2); % {runIdx}{1}: pursuers, {runIdx}{2}: evaders

% 生成初始位置
fprintf('生成初始位置...\n');
for runIdx = 1:numRuns
    % 追捕者初始位置
    pursuerPos = zeros(n_p, 2);
    for i = 1:n_p
        pos = rand(1,2);
        pos(1,1) = pos(1,1) * (xbound(2) - xbound(1)) + xbound(1);
        pos(1,2) = pos(1,2) * (ybound(2) - ybound(1)) + ybound(1);
        pursuerPos(i,:) = pos;
    end
    
    % 逃避者初始位置
    evaderPos = zeros(n_e, 2);
    for i = 1:n_e
        pos = rand(1,2);
        pos(1,1) = pos(1,1) * (xbound(2) - xbound(1)) + xbound(1);
        pos(1,2) = pos(1,2) * (ybound(2) - ybound(1)) + ybound(1);
        evaderPos(i,:) = pos;
    end
    
    initialPositions{runIdx, 1} = pursuerPos;
    initialPositions{runIdx, 2} = evaderPos;
end

% 运行原始策略
fprintf('\n===== 运行原始策略 =====\n');
originalCaptureTimes = zeros(numRuns, 1);
originalFirstCaptureTimes = zeros(numRuns, 1);

for runIdx = 1:numRuns
    fprintf('原始策略 - 运行 #%d/%d\n', runIdx, numRuns);
    
    % 从预生成的初始位置设置
    pursuerPos = initialPositions{runIdx, 1};
    evaderPos = initialPositions{runIdx, 2};
    
    % 运行原始策略（假设Original_strategy已封装为函数）
    [captureTime, firstCaptureTime] = Original_strategy_method(...
        pursuerPos, evaderPos, timestep, timeend, xbound, ybound);
    
    originalCaptureTimes(runIdx) = captureTime;
    originalFirstCaptureTimes(runIdx) = firstCaptureTime;
end

% 运行微分博弈策略
fprintf('\n===== 运行微分博弈策略 =====\n');
differentialCaptureTimes = zeros(numRuns, 1);
differentialFirstCaptureTimes = zeros(numRuns, 1);

for runIdx = 1:numRuns
    fprintf('微分博弈策略 - 运行 #%d/%d\n', runIdx, numRuns);
    
    % 从预生成的初始位置设置
    pursuerPos = initialPositions{runIdx, 1};
    evaderPos = initialPositions{runIdx, 2};
    
    % 运行微分博弈策略（假设Differential_game_strategy已封装为函数）
    [captureTime, firstCaptureTime] = Differential_game_strategy_method(...
        pursuerPos, evaderPos, timestep, timeend, xbound, ybound);
    
    differentialCaptureTimes(runIdx) = captureTime;
    differentialFirstCaptureTimes(runIdx) = firstCaptureTime;
end

% 计算统计指标
% 原始策略
orig_avgCapture = mean(originalCaptureTimes);
orig_stdCapture = std(originalCaptureTimes);
orig_avgFirstCapture = mean(originalFirstCaptureTimes);
orig_stdFirstCapture = std(originalFirstCaptureTimes);

% 微分博弈策略
diff_avgCapture = mean(differentialCaptureTimes);
diff_stdCapture = std(differentialCaptureTimes);
diff_avgFirstCapture = mean(differentialFirstCaptureTimes);
diff_stdFirstCapture = std(differentialFirstCaptureTimes);

% 性能提升计算
captureImprovement = (originalCaptureTimes - differentialCaptureTimes) ./ originalCaptureTimes * 100;
firstCaptureImprovement = (orig_avgFirstCapture - diff_avgFirstCapture) ./ orig_avgFirstCapture * 100;

% ===== 结果可视化 =====
figure('Position', [100, 100, 1200, 800]);

% 1. 总捕获时间对比
subplot(2,2,1);
hold on;
bar(1, orig_avgCapture, 'FaceColor', [0.5, 0.7, 0.9]);
bar(2, diff_avgCapture, 'FaceColor', [0.9, 0.6, 0.5]);
errorbar([1,2], [orig_avgCapture, diff_avgCapture], ...
         [orig_stdCapture, diff_stdCapture], 'k.', 'LineWidth', 1.5);

% 添加数据点
plot(ones(numRuns,1) + 0.1*randn(numRuns,1), originalCaptureTimes, 'ko');
plot(2*ones(numRuns,1) + 0.1*randn(numRuns,1), differentialCaptureTimes, 'ko');

set(gca, 'XTick', [1,2], 'XTickLabel', {'原始策略', '微分博弈策略'});
ylabel('总捕获时间 (s)');
title('总捕获时间对比');
grid on;
legend('平均值', '标准差', '单次运行结果', 'Location', 'northoutside');
hold off;

% 2. 首次捕获时间对比
subplot(2,2,2);
hold on;
bar(1, orig_avgFirstCapture, 'FaceColor', [0.5, 0.7, 0.9]);
bar(2, diff_avgFirstCapture, 'FaceColor', [0.9, 0.6, 0.5]);
errorbar([1,2], [orig_avgFirstCapture, diff_avgFirstCapture], ...
         [orig_stdFirstCapture, diff_stdFirstCapture], 'k.', 'LineWidth', 1.5);

% 添加数据点
plot(ones(numRuns,1) + 0.1*randn(numRuns,1), originalFirstCaptureTimes, 'ko');
plot(2*ones(numRuns,1) + 0.1*randn(numRuns,1), differentialFirstCaptureTimes, 'ko');

set(gca, 'XTick', [1,2], 'XTickLabel', {'原始策略', '微分博弈策略'});
ylabel('首次捕获时间 (s)');
title('首次捕获时间对比');
grid on;
hold off;

% 3. 捕获时间分布对比
subplot(2,2,3);
hold on;
histogram(originalCaptureTimes, 'BinWidth', 5, 'FaceColor', [0.5, 0.7, 0.9], 'EdgeColor', 'none');
histogram(differentialCaptureTimes, 'BinWidth', 5, 'FaceColor', [0.9, 0.6, 0.5], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
xline(orig_avgCapture, '--b', 'LineWidth', 2, 'Label', sprintf('原始均值: %.1f', orig_avgCapture));
xline(diff_avgCapture, '--r', 'LineWidth', 2, 'Label', sprintf('微分博弈均值: %.1f', diff_avgCapture));
xlabel('总捕获时间 (s)');
ylabel('频次');
title('捕获时间分布对比');
legend('原始策略', '微分博弈策略');
grid on;
hold off;

% 添加整体标题
sgtitle('追捕策略性能对比分析', 'FontSize', 16, 'FontWeight', 'bold');

% ===== 统计结果输出 =====
fprintf('\n===== 策略对比统计结果 =====\n');
fprintf('指标\t\t\t原始策略\t\t微分博弈策略\t提升百分比\n');
fprintf('总捕获时间\t%.2f ± %.2f s\t%.2f ± %.2f s\t%.1f%%\n', ...
        orig_avgCapture, orig_stdCapture, ...
        diff_avgCapture, diff_stdCapture, ...
        mean(captureImprovement));
    
fprintf('首次捕获时间\t%.2f ± %.2f s\t%.2f ± %.2f s\t%.1f%%\n', ...
        orig_avgFirstCapture, orig_stdFirstCapture, ...
        diff_avgFirstCapture, diff_stdFirstCapture, ...
        mean(firstCaptureImprovement));

% 统计显著性检验（配对t检验）
[h_capture, p_capture] = ttest(originalCaptureTimes, differentialCaptureTimes);
[h_first, p_first] = ttest(originalFirstCaptureTimes, differentialFirstCaptureTimes);

fprintf('\n===== 统计显著性检验 =====\n');
fprintf('总捕获时间差异显著性: p = %.8f (%s)\n', ...
        p_capture, ternary(h_capture, '显著', '不显著'));
fprintf('首次捕获时间差异显著性: p = %.8f (%s)\n', ...
        p_first, ternary(h_first, '显著', '不显著'));

% 辅助函数
function result = ternary(condition, trueStr, falseStr)
    if condition
        result = trueStr;
    else
        result = falseStr;
    end
end