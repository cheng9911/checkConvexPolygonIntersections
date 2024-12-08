clc,clear;close all
global ptCloud step_len pointUser  last_pointUser last_pointUser1 delta pointUser1;

delta = [0, 0];

ptCloud = pcread("step5.ply");

step_len =5;

sp = [0, 0];
tp = [0, 0];

figure;
% 第一个点和最后一个点相连


plot([ptCloud.Location(:, 1);ptCloud.Location(1, 1)],[ptCloud.Location(:,2);ptCloud.Location(1, 2)]);

hold on;

pointUser = [-37.9749489-0.1, 146];
pointUser1= [-37.9749489-0.1, 150];
last_pointUser=pointUser;

last_pointUser1=pointUser1;


scatter(pointUser(1), pointUser(2), 10, 'filled', 'r');

set(gcf, 'KeyPressFcn', @movePoint);

function movePoint(~, event)
    global ptCloud step_len pointUser delta last_pointUser pointUser1 last_pointUser1;
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


    [x, p] = getNextPoint;

    
    if ~x
        if ~isempty(p)
            pointUser = p;
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
    % 标注A12
    text(pointUser(1), pointUser(2), 'A2 B2');
    plot([pointUser1(1),pointUser(1)],[pointUser1(2),pointUser(2)],'r');
    text(last_pointUser1(1), last_pointUser(2), 'A1 B1');
    plot([last_pointUser1(1),last_pointUser(1)],[last_pointUser1(2),last_pointUser(2)],'b');
    % plot([pointUser1(1),last_pointUser1(1)],[pointUser1(2),last_pointUser1(2)],'g');
    % plot([pointUser(1),last_pointUser(1)],[pointUser(2),last_pointUser(2)],'g');
    last_pointUser=pointUser;
    last_pointUser1=pointUser1;
    hold on;
    
    text(10, 180, txt)

end



function [x, p] = getNextPoint

    global ptCloud step_len pointUser delta  pointUser1;

    x = true;
    p = [];
    num_pts = size(ptCloud.Location, 1);
    % segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)]; % 定义所有线段
    segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)];  % 原始相邻点的线段
    last_segment = [ptCloud.Location(end, :), ptCloud.Location(1, :)];      % 最后一个点与第一个点的线段
    segments = [segments; last_segment];                                    % 合并两部分

    % 定义用户移动的线段 A1 和 A2
    A1 = pointUser;
    A2 =  A1  + delta;
    B1=pointUser1;
    B2=B1+delta;
    % 检查线段 A1A2 是否与多边形相交
    % 调用 findIntersectionsEfficient 函数找到交点
    % [valid_indices1, time_taken1] = findIntersections(A1, A2, segments);
    [intersection_points1, valid_indices1,time_taken1,is_inside1] = findIntersections_onepoint(A1, A2, segments);
    % [valid_indices2, time_taken2] = findIntersections(B1, B2, segments);
    [intersection_points2, valid_indices2,time_taken2,is_inside2] = findIntersections_onepoint(B1, B2, segments);
    % [valid_indices3, time_taken3] = findIntersections(A2, B2, segments);
    [intersection_points3, valid_indices3,time_taken3,is_inside3] = findIntersections_onepoint(A2, B2, segments);
    % 三个只要有一个相交
    % if(~isempty(valid_indices1) || ~isempty(valid_indices2) || ~isempty(valid_indices3))
    %     x=false;
    %     valid_indices=[valid_indices1;valid_indices2;valid_indices3];
    % %    去除重复的点
    %     valid_indices=unique(valid_indices);
    % else
    %     x=false;
        

    % end
    % 三条只有A1-A2相交 或者三条只有B1 B2 相交
    if((~isempty(valid_indices1) && isempty(valid_indices2) && isempty(valid_indices3))  )

       if(is_inside1)
            x=true;
        else
            x=false;
        end
       
    elseif (isempty(valid_indices1) && ~isempty(valid_indices2) && isempty(valid_indices3))
        disp("B1 B2")

    elseif(isempty(valid_indices1) && isempty(valid_indices2) && ~isempty(valid_indices3))
        disp("A2 B2")
    elseif(~isempty(valid_indices1) && ~isempty(valid_indices2) && ~isempty(valid_indices3))
        disp("A1 A2 B1 B2 A2 B2")
    elseif(isempty(valid_indices1) && ~isempty(valid_indices2) && ~isempty(valid_indices3))
        disp("B1 B2 A2 B2")
        % 交点的边取出，find_line_segment_intersection
        % 取delta的方向矢量
        d=delta/norm(delta);
        
       

        % x=false;
    end
    % 



    
end

function [valid_indices, time_taken] = findIntersections(A1, A2, segments)
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
        
        if ~isempty( )
            for(i=1:length(valid_indices))
                disp("valid_indices ")
                valid_indices(i)
                disp("********* ")
                
            end
            
        end
    end
    
    time_taken = toc;  % 结束计时并返回计算时间
end

function [intersection_points, valid_indices,time_taken,is_inside] = findIntersections_onepoint(A1, A2, segments)
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
    % if is_inside
        
    %     disp('点在多边形内');
    % else
        
    %     disp('点在多边形外');
    % end
    
    
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
% 检测两条线段是否相交
function [is_intersect, intersect_point] = check_intersection(p1, p2, q1, q2)
    % 向量法检测线段相交
    intersect_point = [];
    is_intersect = false;
    
    % 计算方向向量
    r = p2 - p1;
    s = q2 - q1;
    
    % 计算叉积
    rxs = r(1) * s(2) - r(2) * s(1);
    qp = q1 - p1;
    qpxr = qp(1) * r(2) - qp(2) * r(1);
    
    if rxs == 0 && qpxr == 0
        % 共线，不相交
        return;
    elseif rxs == 0 && qpxr ~= 0
        % 平行，不相交
        return;
    end
    
    % 计算参数
    t = (qp(1) * s(2) - qp(2) * s(1)) / rxs;
    u = (qp(1) * r(2) - qp(2) * r(1)) / rxs;
    
    if t >= 0 && t <= 1 && u >= 0 && u <= 1
        % 两条线段相交，计算交点
        intersect_point = p1 + t * r;
        is_intersect = true;
    end
end


