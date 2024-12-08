clc,clear;close all
global ptCloud step_len pointUser  last_pointUser delta;

delta = [0, 0];

ptCloud = pcread("step5.ply");

step_len =10;

sp = [0, 0];
tp = [0, 0];

figure;
% 第一个点和最后一个点相连


plot([ptCloud.Location(:, 1);ptCloud.Location(1, 1)],[ptCloud.Location(:,2);ptCloud.Location(1, 2)]);

hold on;

pointUser = [-37.9749489-0.1, 146];
pointUser1= [-37.9749489-0.1, 150];
last_pointUser=pointUser;



scatter(pointUser(1), pointUser(2), 10, 'filled', 'r');

set(gcf, 'KeyPressFcn', @movePoint);

function movePoint(~, event)
    global ptCloud step_len pointUser delta pointUser1 last_pointUser;
    clf;
    
    switch event.Key
        case 'uparrow'
            txt = "\uparrow UP";
            delta = [0,  step_len];   
        case 'downarrow'
            txt = "\downarrow DOWN";
            delta = [0, -step_len];   
        case 'leftarrow'
            txt = "\leftarrow LEFT";
            delta = [-step_len, 0];   
        case 'rightarrow'
            txt = "\rightarrow RIGHT";
            delta = [ step_len, 0];   
    end


    [x, p,t] = getNextPoint;

    
    if ~x
        if ~isempty(p)
            pointUser = pointUser+t*delta;
            pointUser1 = pointUser1+t*delta;
            
            disp(['交点为：',num2str(p(1)),',' ,num2str(p(2))]);
            
            scatter(p(1), p(2), 100, 'x', 'b');
            hold on;

        end  
    else
        pointUser = pointUser + delta;
        pointUser1 = pointUser1 + delta;

    end
    
    % 定义上一时刻的点

    % plot(ptCloud.Location(:, 1),ptCloud.Location(:,2));
    plot([ptCloud.Location(:, 1);ptCloud.Location(1, 1)],[ptCloud.Location(:,2);ptCloud.Location(1, 2)]);
    hold on;
    scatter(pointUser(1), pointUser(2), 10, 'filled', 'r');
    hold on;
    scatter(pointUser1(1), pointUser1(2), 10, 'filled', 'g');
    plot([pointUser1(1),pointUser(1)],[pointUser1(2),pointUser(2)],'r');
    % last_pointUser=pointUser;
    hold on;
    
    text(10, 180, txt)

end



function [x, p] = getNextPoint

    global ptCloud step_len pointUser pointUser1 delta ;

    x = true;
    p = [];

    % 定义用户移动的线段 A1 和 A2
    A1 = pointUser + delta;
    A2 = pointUser1 + delta;                                                               

    
    num_pts = size(ptCloud.Location, 1);
    % segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)]; % 定义所有线段       
    segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)];  % 原始相邻点的线段
    last_segment = [ptCloud.Location(end, :), ptCloud.Location(1, :)];      % 最后一个点与第一个点的线段
    segments = [segments; last_segment];                                    % 合并两部分

    % 调用 findIntersectionsEfficient 函数找到交点
    [intersection_points, time_taken,is_inside] = findIntersections(A1, A2, segments);
    disp(["time",num2str(time_taken)])
%    打印intersection_points行数
    disp(["size",num2str(size(intersection_points, 1))])
    
    if ~isempty(intersection_points)
        p = intersection_points(1, 1:2);  % 使用找到的第一个交点
        
       
        if is_inside
            x = true;
            
        else
            x = false;
            
        end
        
    end
end



function [intersection_points, time_taken, is_inside] = findIntersections(A1, A2, segments)
    % A1, A2 是线段 A 的起点和终点
    % segments 是一个 N x 6 的矩阵，每一行代表一条线段 [x1 y1 z1 x2 y2 z2]
    
    % 定义用户移动的线段 A1 和 A2
    A1 = [A1, 0];  % 确保 A1 是 3D 坐标
    A2 = [A2, 0];  % 确保 A2 是 3D 坐标
    r = A2 - A1;  % 线段 A 的向量 r

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
        valid_indices = find((t >= -1e-5 & t <= 1+1e-5) & (u >= -1e-5 & u <= 1+1e-5));
        
        if ~isempty(valid_indices)
            % 如果存在相交的线段，取t最小的那个
            if(length(valid_indices)>1)
                t(valid_indices)
                [t_min,t_min_index]= min(t(valid_indices));
                intersection_points = A1 + t_min .* r;
                is_inside=false
            else
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
                
                % 判断 `A2` 是否在图形内：根据交点数的奇偶性
        is_inside = mod(num_intersections, 2) == 1;  % 奇数为内，偶数为外
    end
    % t_negative_indices = find(t >1-1e-5 & (u >= -1e-5 & u <= 1));
    % num_intersections = length(t_negative_indices);  % 交点数
            
    %         % 判断 `A2` 是否在图形内：根据交点数的奇偶性
    % is_inside = mod(num_intersections, 2) == 1;  % 奇数为内，偶数为外
    if is_inside
        
        disp('点在多边形内');
    else
        
        disp('点在多边形外');
    end
    
    
    time_taken = toc;  % 结束计时并返回计算时间
end



% 处理情况1：AC 和 BD 都未与边界相交
function handleCase1(C, D)
    % 直接步进
    disp('Case 1: CD 没有相交，直接步进');
end

% 处理情况2：AC 或 BD 与边界相交
function handleCase2(A, B, C, D, intersect_AC, intersect_BD, boundary)
    % 根据AC或BD与边界相交的情况，调整步进长度
    disp('Case 2: AC 或 BD 与边界相交，调整步进');
    
    if intersect_AC
        % AC相交的处理逻辑，例如减少步进
        disp('AC 与边界相交');
    end
    
    if intersect_BD
        % BD相交的处理逻辑，例如减少步进
        disp('BD 与边界相交');
    end
end

% 处理情况3：AC 和 BD 都与边界相交
function handleCase3(A, B, C, D, boundary)
    % 计算AC和BD之间的最短距离，调整步进
    disp('Case 3: AC 和 BD 都与边界相交');
    
    % 找到AC和BD的相交线段序列，并计算最短距离
    % 这里可以使用几何算法来计算线段之间的最短距离
    shortestDistance = calculateShortestDistance(A, B, C, D, boundary);
    
    % 步进长度等于最短距离
    disp(['最短步进距离: ', num2str(shortestDistance)]);
end

% 计算两条线段是否相交
function isIntersect = isLineSegmentIntersect(P1, P2, Q1, Q2)
    % 判断两条线段 P1P2 和 Q1Q2 是否相交
    % 可以使用叉积法或向量投影法等几何方法进行相交判断
    isIntersect = false;
    
    % 叉积法判断线段相交
    % ...
end