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

pointUser = [5, 146];
pointUser1= [5, 150];
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
            pointUser = pointUser+p*delta;
            pointUser1 = pointUser1 + p*delta;
            % disp(['交点为：',num2str(p(1)),',' ,num2str(p(2))]);
            
            % scatter(p(1), p(2), 100, 'x', 'b');
            % hold on;

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

    global ptCloud step_len pointUser delta pointUser1 ;

    x = true;
    p = [];

    % 定义用户移动的线段 A1 和 A2
    A1 = pointUser;
    A2 =pointUser1   ;
    B1=pointUser+delta;
    B2=pointUser1+delta;
    % 离散化线段
    num_points = 20;
    t_vals = linspace(0, 1, num_points)';
    P1 = A1 + t_vals * (A2 - A1);
    P2 = B1 + t_vals * (B2 - B1);
    T=[];
    points=[];
    num_pts = size(ptCloud.Location, 1);
    % segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)]; % 定义所有线段
    segments = [ptCloud.Location(1:end-1, :), ptCloud.Location(2:end, :)];  % 原始相邻点的线段
    last_segment = [ptCloud.Location(end, :), ptCloud.Location(1, :)];      % 最后一个点与第一个点的线段
    segments = [segments; last_segment];                                    % 合并两部分
    time_all=0;
    inside=0;
    for(i=1:num_points)
        [intersection_points, time_taken, is_inside,min_t] = findIntersections(P1(i, :), P2(i, :), segments);
        time_all=time_all+time_taken;
        if ~isempty(intersection_points)
            T=[T,min_t];
            points=[points,intersection_points];
           
        end
    end
    % disp("time_all");
   time_all
    [intersection_points, time_taken, is_inside,min_t] = findIntersections(A1, A2, segments);
    disp("A1 A2");
    is_inside
    % if ~isempty(intersection_points)
    %     T=[T,min_t];
    %     points=[points,intersection_points];
    % end

    [intersection_points, time_taken, is_inside,min_t] = findIntersections(B1, B2, segments);
    % if ~isempty(intersection_points)
    %     T=[T,min_t];
    %     points=[points,intersection_points];
    % end
    disp("B1 B2");
    is_inside
    min_t
    
    if ~isempty(points)
        % p = min(T)
         % 使用找到的第一个交点 
        
        
    end
    if is_inside
        x = true;
        p=1
        
    else
        x = false;
        p = min(T)
    end
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
        valid_indices = find((t > -1e-7 & t <= 1+1e-7) & (u >= -1e-7 & u <= 1+1e-7));
        
        if ~isempty(valid_indices)
            % 如果存在相交的线段，取t最小的那个
            if(length(valid_indices)>1)
            
                [t_min,t_min_index]= min(t(valid_indices));
                intersection_points = A1 + t_min .* r;
                is_inside=false;
            else
                t_min=t(valid_indices);
                % disp("t_min");
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
        % num_intersections
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

function [nearest_point, time_taken, is_inside,min_t] = findNearestBoundary(A1, A2, segments, num_points, delta)
    % A1, A2 是线段 A 的起点和终点
    % segments 是一个 N x 6 的矩阵，每一行代表一条线段 [x1 y1 z1 x2 y2 z2]
    B1=[A1+delta,0];
    B2=[A2+delta,0];
    % 定义用户移动的线段 A1 和 A2
    A1 = [A1, 0];  % 确保 A1 是 3D 坐标
    A2 = [A2, 0];  % 确保 A2 是 3D 坐标
    r = A2- A1;  % 线段 A 的向量 r
    

    % 将 A1 和 A2 离散为 num_points 个点
    t_vals = linspace(0, 1, num_points)';
    P1= A1 + t_vals * r;  % 离散后的点的集合 (num_points x 3)
    P2=B1 + t_vals * r;
    P=P2-P1;

    num_segments = size(segments, 1);  % 矩阵中的线段数量
    C = segments(:, 1:2);  % 提取每条线段的起点 C
    C = [C, zeros(num_segments, 1)];  % 扩展为 3D 坐标
    D = segments(:, 4:5);  % 提取每条线段的终点 D
    D = [D, zeros(num_segments, 1)];  % 扩展为 3D 坐标
    s = D - C;  % 当前线段的向量 s (矩阵操作)

    tic;  % 开始计时

    % 计算 r 和 s 的叉积 (批量计算)
    rxs = cross(repmat(r, num_segments, 1), s, 2);
    
    % 找到 rxs 不为 0 的线段 (即不平行的线段)
    non_parallel_indices = find(vecnorm(rxs, 2, 2) > 1e-8);  % 叉积非零的索引

    nearest_point = [];
    min_t = inf;
    is_inside = true;  % 假设点在多边形内

    if ~isempty(non_parallel_indices)
        C_non_parallel = C(non_parallel_indices, :);
        s_non_parallel = s(non_parallel_indices, :);
        for i = 1:num_points
            P_i = P(i, :);  % 取出当前点 P_i
            rxs= cross(repmat(P_i, num_segments, 1), s, 2);
            
            rxs_non_parallel = rxs(non_parallel_indices, :);

        
            
            
            % 计算 t 和 u (批量计算)
            C_minus_P_i = C_non_parallel - repmat(P1(i), size(C_non_parallel, 1), 1);
            t = dot(cross(C_minus_P_i, s_non_parallel, 2), rxs_non_parallel, 2) ./ vecnorm(rxs_non_parallel, 2, 2).^2;
            u = dot(cross(C_minus_P_i, repmat(P_i, size(C_non_parallel, 1), 1), 2), rxs_non_parallel, 2) ./ vecnorm(rxs_non_parallel, 2, 2).^2;

            % 筛选满足 0 <= t <= 1 和 0 <= u <= 1 的线段 (找到相交的线段)
            valid_indices = find((t >= -1e-5 & t <= 1+1e-5) & (u >= -1e-5 & u <= 1+1e-5));

            if ~isempty(valid_indices)
                [t_min, t_min_idx] = min(t(valid_indices));
                if t_min < min_t
                    min_t = t_min;
                    t_min
                    nearest_point = P1(i)+t_min*P_i;  % 更新最近交点
                    is_inside = false;  % 找到相交，说明在多边形外
                end
            end
        end
    end

    time_taken = toc;  % 结束计时并返回计算时间
end
function [nearest_point, time_taken, is_inside, min_t] = findNearestBoundary1(A1, A2, segments, num_points, delta)
    % A1, A2 是线段 A 的起点和终点
    A1 = [A1, 0];  % 确保 A1 是 3D 坐标
    A2 = [A2, 0];  % 确保 A2 是 3D 坐标
    r = A2 - A1;  % 线段 A 的向量 r
    
    % 将 A1 和 A2 离散为 num_points 个点
    t_vals = linspace(0, 1, num_points)';  % 生成离散点的参数
    P1 = A1 + t_vals * r;  % 离散后的点的集合 (num_points x 3)
    B1 = A1 + [delta,0];  % 步进后的起点
    B2 = A2 + [delta,0];  % 步进后的终点
    P2=B1 + t_vals * (B2-B1);  % 离散后的点的集合 (num_points x 3)
    r_step = P2 - P1;  % 步进后的向量

    % 提取边界线段的起点和终点
    C = segments(:, 1:3);  % 起点
    D = segments(:, 4:6);  % 终点
    s = D - C;  % 线段的向量

    tic;  % 开始计时

    nearest_point = [];  % 最近的交点
    min_t = -1;  % 初始化最小 t
    is_inside = true;  % 假设点在多边形内

    % 计算 r 和 s 的叉积 (批量计算)
    rxs = cross(repmat(r_step(1,:), size(segments, 1), 1), s, 2);
    
    % 找到 rxs 不为 0 的线段 (即不平行的线段)
    non_parallel_indices = find(vecnorm(rxs, 2, 2) > 1e-8);  % 叉积非零的索引

    if ~isempty(non_parallel_indices)
        C_non_parallel = C(non_parallel_indices, :);
        s_non_parallel = s(non_parallel_indices, :);
        

        for i = 1:num_points
            P_i = P1(i, :);  % 当前离散点 P_i
            rxs=cross(repmat(r_step(i,:), size(segments, 1), 1), s, 2);
            rxs_non_parallel = rxs(non_parallel_indices, :);
            
            % 计算 t 和 u (批量计算)
            C_minus_P_i = C_non_parallel - repmat(P_i, size(C_non_parallel, 1), 1);
            t = dot(cross(C_minus_P_i, s_non_parallel, 2), rxs_non_parallel, 2) ./ vecnorm(rxs_non_parallel, 2, 2).^2;
            u = dot(cross(C_minus_P_i, repmat(r_step(i,:), size(C_non_parallel, 1), 1), 2), rxs_non_parallel, 2) ./ vecnorm(rxs_non_parallel, 2, 2).^2;

            % 筛选满足 0 <= t <= 1 和 0 <= u <= 1 的线段 (找到相交的线段)
            valid_indices = find((t >= -1e-5 & t <= 1+1e-5) & (u >= -1e-5 & u <= 1+1e-5));

            if ~isempty(valid_indices)
                [t_min, t_min_idx] = min(t(valid_indices));
                T=[T,t_min];
                if t_min < min_t
                    min_t = t_min;  % 更新最小 t
                    nearest_point = P_i + t_min * r_step;  % 更新最近交点
                    is_inside = false;  % 找到交点，说明在多边形外
                end
            end
        end
    end

    % 如果没有找到交点，默认 P 是在多边形内
    if isempty(nearest_point)
        nearest_point = A2;  % 如果在内，则返回 A2
        is_inside = true;  % 在多边形内
    end

    time_taken = toc;  % 结束计时并返回计算时间
    
    if is_inside
        disp('点在多边形内');
    else
        disp('点在多边形外');
    end
end
