classdef Pursuer < Robot
    %PURSUER 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        target % 追踪的目标
        targetIsAdjacent    % 目标所在cell与pursuer所在cell是否邻接
        max_speed = 2.0;    % 最大速度（需设置）
        last_decision_time = 0; % 上次决策时间
        strategy_mode = 'direct'; % 或 'boundary'
        last_prediction = [];     % 上次预测位置
        prediction_time = 0;      % 上次预测时间
        last_capture_progress = 0;
    end
    
    methods
        function obj = Pursuer(pos)
            obj = obj@Robot(pos);
            obj.position = pos;
            obj.velocity = [0, 0]; % 初始化速度
        end
        
        function obj = setTarget(evader)
            obj.target = evader;
        end
        
        function obj = calculateVelocity(obj)
            % 判断target是否已经dead
            
            % 判断目标cell与pursuer所在cell是否邻接
            if obj.targetIsAdjacent
                % 计算边界
                sharedVertices = getSharedBound(obj.voronoi_cell, obj.target.voronoi_cell); % 计算voronoi boundary
                % compute u_p^j
                centroidOfBound = mean(sharedVertices, 1);
                v = (centroidOfBound - obj.position) / norm(centroidOfBound - obj. position);
            else
                % if no neighbor evader exists, move directly towards nearest evader
                v = (obj.target.position - obj.position) / norm(obj.target.position - obj.position);
            end        
            obj.targetIsAdjacent = false;   % 刷新，以防止上一目标邻接但下一目标不邻接。
            obj = obj.setVelocity(v);
            if isempty(obj.velocity)
                error('Velocity not initialized!');
            end
        end

    end
end

