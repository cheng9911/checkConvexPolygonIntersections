% 问题描述，知道CD和AB平行，知道CD上的一点E求解CD和AB的平移距离，
clc,clear
% 使用示例
A = [-30, 1.0909817e+02]; % 线段AB的起点
B = [-25, 1.1309817e+02]; % 线段AB的终点
E = [1, 2]; % 线段CD上的点
d = [-1, 0]; % 平移方向向量
% [distance,point1,point2] = calculateShortestDistance(A, B, E, [E(1)+0.1, E(2)+0.2]); % 计算平移距离
ptCloud = pcread("step5.ply");
segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)];  % 原始相邻点的线段
last_segment = [ptCloud.Location(end, :), ptCloud.Location(1, :)];      % 最后一个点与第一个点的线段
segments = [segments; last_segment];      
[intersection_points, time_taken, is_inside,t_min] = findIntersections(A, B, segments)
is_inside
t_min

% [intersections,distance] = find_line_segment_intersection(A, B, E, -d);

% 绘制曲线
% figure(1);
% plot([A(1), B(1)], [A(2), B(2)], 'r-'); % 画线段AB
% hold on;
% plot(E(1), E(2), 'bo'); % 画点E
% % 绘制平移后的线段
% if ~isempty(intersections)
%     plot(intersections(1), intersections(2), 'ro'); % 画交点
%     plot([E(1), intersections(1)], [E(2), intersections(2)], 'b--'); % 画线段CE
%     fprintf('平移后的线段长度: %.4f\n', distance);
  
%     plot([A(1)+distance*d(1), B(1)+distance*d(1)], [A(2)+distance*d(2), B(2)+distance*d(2)], 'g-'); % 画平移后的线段
% end
% plot(point1(1), point1(2), 'go'); % 画最短距离的点
% plot(point2(1), point2(2), 'go'); % 画最短距离的点



function [intersection_point,distance ]= find_line_segment_intersection(A, B, P, d)
    % A, B: 线段的端点
    % P: 向量的起点
    % d: 向量的方向

    % 线段的方向
    AB = B - A;

    % 设置方程：A + s * (B - A) = P + t * d
    % 变换为：s * AB - t * d = P - A
    % 形成线性方程组 AX = B

    A_eq = [AB(1), -d(1); AB(2), -d(2)];
    B_eq =(P - A)';

    % 求解线性方程组
    if rank(A_eq) == 2
        solution = A_eq \ B_eq; % 解方程 AX = B
        s = solution(1);
        t = solution(2);
        
        % 判断 s 和 t 的值
        if (s >= 0-1e-5 && s <= 1+1e-5) && (t >= 0+1e-5) % 线段内且向量的 t ≥ 0
            intersection_point = A + s * AB; % 交点
            fprintf('交点坐标: (%.4f, %.4f)\n', intersection_point(1), intersection_point(2));
            % 平移距离
            distance = norm(intersection_point - P);
        else
            intersection_point = []; % 没有交点
            distance = 0;
            fprintf('没有交点\n');
        end
    else
        intersection_point = []; % 方程无解
        distance = 0;
        fprintf('没有交点\n');
    end
end
% 求两个线段之间最短距离的点
function [shortestDistance,Q,A] = calculateShortestDistance(A, B, C, D)
    % A, B: 线段1的端点
    % C, D: 线段2的端点

    % 计算线段1的方向
    AB = B - A;

    % 计算线段2的方向
    CD = D - C;

    % 计算两个线段之间的向量
    AC = C - A;

    % 计算线段1和线段2的方向向量的点积
    dot_AB_CD = dot(AB, CD);
    dot_AB_AC = dot(AB, AC);
    dot_CD_AC = dot(CD, AC);
    dot_CD_AB = dot(CD, AB);
    dot

    % 计算两个线段之间的最短距离
    s = (dot_AB_AC * dot_CD_CD - dot_CD_AC * dot_CD_AB) / (dot_AB_CD * dot_CD_CD - dot_CD_AB * dot_CD_AB);
    t = (dot_AB_AC + s * dot_CD_AB) / dot_CD_CD;

    % 计算最短距离的点
    P = A + s * AB;
    Q = C + t * CD;

    % 计算最短距禿
    shortestDistance = norm(P - Q);
end



function [intersection_points, time_taken, is_inside,t_min] = findIntersections(A1, A2, segments)
    % A1, A2 是线段 A 的起点和终点
    % segments 是一个 N x 6 的矩阵，每一行代表一条线段 [x1 y1 z1 x2 y2 z2]
    
    % 定义用户移动的线段 A1 和 A2
    A1 = [A1, 0];  % 确保 A1 是 3D 坐标
    A2 = [A2, 0];  % 确保 A2 是 3D 坐标
    r = A2 - A1;  % 线段 A 的向量 r
    t_min=10000;
    num_segments = size(segments, 1);  % 矩阵中的线段数量
    C = segments(:, 1:2);  % 提取每条线段的起点 C
    C = [C, zeros(num_segments, 1)];  % 扩展为 3D 坐标
    D = segments(:, 4:5);  % 提取每条线段的终点 D
    D = [D, zeros(num_segments, 1)];  % 扩展为 3D 坐标
    s = D - C;  % 当前线段的向量 s (矩阵操作)
    num_intersections=0;
    is_inside=true;
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
        valid_indices = find((t > 0 & t <= 1-1e-7) & (u >= 0 & u <= 1));
        
        if ~isempty(valid_indices)
            % 如果存在相交的线段，取t最小的那个
            if(length(valid_indices)>1)
            
                [t_min,t_min_index]= min(t(valid_indices));
                intersection_points = A1 + t_min .* r;
                is_inside=false;
            else
                t_min=t(valid_indices);
                disp("t_min");
                % t_min
                is_inside=false;
                intersection_points = A1 + t(valid_indices) .* repmat(r, numel(valid_indices), 1);
            end
            

            % 计算所有相交的交点
%             intersection_points = A1 + t(valid_indices) .* repmat(r, numel(valid_indices), 1);
            % 计算 `t 》1` 且 `0 <= u <= 1` 的部分，用于判断 `A2` 是否在多边形内
            
        end
    end
    if(is_inside)
        % 计算 `t 》1` 且 `0 <= u <= 1` 的部分，用于判断 `A2` 是否在多边形内
        
        t_negative_indices = find(t >1-1e-5 & (u >= -1e-5 & u <= 1));
        num_intersections = length(t_negative_indices);  % 交点数
        num_intersections
                % 判断 `A2` 是否在图形内：根据交点数的奇偶性
        is_inside = mod(num_intersections, 2) == 1;  % 奇数为内，偶数为外
    end
    % t_negative_indices = find(t >1-1e-5 & (u >= -1e-5 & u <= 1));
    % num_intersections = length(t_negative_indices);  % 交点数
            
    %         % 判断 `A2` 是否在图形内：根据交点数的奇偶性
    % is_inside = mod(num_intersections, 2) == 1;  % 奇数为内，偶数为外
    % if is_inside
        
    %     disp('点在多边形内');
    % else
        
    %     disp('点在多边形外');
    % end
    
    
    time_taken = toc;  % 结束计时并返回计算时间
end