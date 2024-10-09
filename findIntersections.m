function [intersection_points, time_taken] = findIntersections(A1, A2, segments)
    % A1, A2 是线段 A 的起点和终点
    % segments 是一个 N x 6 的矩阵，每一行代表一条线段 [x1 y1 z1 x2 y2 z2]
    
    r = A2 - A1;  % 线段 A 的向量 r
    num_segments = size(segments, 1);  % 矩阵中的线段数量
    
    C = segments(:, 1:3);  % 提取每条线段的起点 C
    D = segments(:, 4:6);  % 提取每条线段的终点 D
    
    s = D - C;  % 当前线段的向量 s (矩阵操作)
    
    tic;  % 开始计时
    
    % 计算 r 和 s 的叉积 (批量计算)
    rxs = cross(repmat(r, num_segments, 1), s, 2);
    
    % 找到 rxs 不为 0 的线段 (即不平行的线段)
    non_parallel_indices = find(vecnorm(rxs, 2, 2) > 1e-8);  % 叉积非零的索引
    
    % 初始化交点存储
    intersection_points = [];
    
    % 如果存在非平行的线段，计算 t 和 u
    if ~isempty(non_parallel_indices)
        C_non_parallel = C(non_parallel_indices, :);
        s_non_parallel = s(non_parallel_indices, :);
        rxs_non_parallel = rxs(non_parallel_indices, :);
        
        % 计算 t 和 u (批量计算)
        C_minus_A1 = C_non_parallel - repmat(A1, size(C_non_parallel, 1), 1);
        t = dot(cross(C_minus_A1, s_non_parallel, 2), rxs_non_parallel, 2) ./ vecnorm(rxs_non_parallel, 2, 2).^2;
        u = dot(cross(C_minus_A1, repmat(r, size(C_non_parallel, 1), 1), 2), rxs_non_parallel, 2) ./ vecnorm(rxs_non_parallel, 2, 2).^2;
        
        % 筛选满足 0 <= t <= 1 和 0 <= u <= 1 的线段 (找到相交的线段)
        valid_indices = find((t >= 0 & t <= 1) & (u >= 0 & u <= 1));
        
        if ~isempty(valid_indices)
            % 计算所有相交的交点
            intersection_points = A1 + t(valid_indices) .* repmat(r, numel(valid_indices), 1);
        end
    end
    
    time_taken = toc;  % 结束计时并返回计算时间
end