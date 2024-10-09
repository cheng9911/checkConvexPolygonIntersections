global ptCloud step_len pointUser last_pointUser delta;

delta = [0, 0];

ptCloud = pcread("step5.ply");

step_len = 5;

sp = [0, 0];
tp = [0, 0];

figure;

plot(ptCloud.Location(:, 1),ptCloud.Location(:,2));

hold on;

pointUser = [-37.9749489-0.1, 146];
last_pointUser=pointUser;


scatter(pointUser(1), pointUser(2), 10, 'filled', 'r');

set(gcf, 'KeyPressFcn', @movePoint);

function movePoint(~, event)
    global ptCloud step_len pointUser delta last_pointUser;
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


    %如果按了Up，则更新pointUserNext
%     pointUserNext(1) = pointUser(1);
%     pointUserNext(2) = pointUser(2) + step_len;

%     if ~is_in_2d_polygon
%         %p = intersection_point(pointUser(1), pointUser(2), pointUser(1), pointUser(2) - step_len, sp(1), sp(2), tp(1), tp(2));
%         p = polyxpoly(pointUser, [pointUser(1), pointUser(2) - step_len], sp, tp);
%         if ~isempty(p)
%             pointUser(2) = p(2);
%         else
%             pointUser(2) = pointUser(2) + step_len; 
%         end
%     end

    [x, p] = getNextPoint;
    
    if ~x
        if ~isempty(p)
            pointUser = p;
            disp(['交点为：', p]);
            scatter(p(1), p(2), 100, 'x', 'b');
            hold on;

        end  
    else
        pointUser = pointUser + delta;
    end
    

    plot(ptCloud.Location(:, 1),ptCloud.Location(:,2));
    hold on;
    scatter(last_pointUser(1), last_pointUser(2), 10, 'filled', 'r');
    scatter(pointUser(1), pointUser(2), 10, 'filled', 'r');
    plot([last_pointUser(1),pointUser(1)],[last_pointUser(2),pointUser(2)],'r');
    last_pointUser = pointUser;
  
    text(10, 180, txt)

end



function [x, p] = getNextPoint

    global ptCloud step_len pointUser delta;

    x = false;
    p = [];

    pointUserNext = pointUser + delta;
    px = pointUserNext(1);
    py = pointUserNext(2);

    angle_sum = 0;

    dis = 0;
    dis_min = 10000;

    j = size(ptCloud.Location, 1);

    for i = 1:size(ptCloud.Location, 1)
        sx = ptCloud.Location(i, 1);
        sy = ptCloud.Location(i, 2);
        tx = ptCloud.Location(j, 1);
        ty = ptCloud.Location(j, 2);

        A = sy - ty;
        B = tx - sx;
        C = (sx-tx)*ty-(sy-ty)*tx;

        ab = A^2 + B^2;

        [x, y] = polyxpoly([pointUser(1), pointUserNext(1)], [pointUser(2), pointUserNext(2)], [sx, tx], [sy, ty]);
        if ~isempty(x) && ~isempty(y)

            if (x >= min([sx, tx]) && y >= min(sy, ty)) || (x <= max(sx, tx) && y <= max(sy, ty))
                p = [x, y];
                
                disp("point: ");
                disp(p);
                disp('i: ');
                disp(i);
                disp('j: ');
                disp(j);
                
%                 line([sx, tx], [sy, ty], 'Color','red','LineStyle','--');
                hold on;

            end
        end

        if ab >= 0.002^2
            
            dis = abs(A*px + B*py + C) / sqrt(ab);
            
            if dis < 0.002
                if (px > sx && px < tx) || (px < sx && px > tx)
                 x = true;
                 return;
                end
            else
                if dis < dis_min
                    dis_min = dis;
                end
            end

            angle = atan2(sy-py, sx-px)-atan2(ty-py,tx-px);
            if angle >= pi
                angle = angle - pi * 2;
            elseif angle <= -pi
                angle = angle + pi * 2;
            end

            angle_sum = angle_sum + angle;
            j = i;

        end

    end

    x = abs(angle_sum - pi * 2) < 1e-4;
    return;

end



% 没用上
function p = intersection_point(x11, y11, x12, y12, x21, y21, x22, y22)
    % [x11, y11] 第一条直线第一点
    % [x12, y12] 第一条直线第二点
    % [x21, y21] 第二条直线第一点
    % [x22, y22] 第二条直线第二点

    A1 = y12 - y11;
    B1 = x11 - x12;
    C1 = A1 * x11 + B1 * y11;

    A2 = y22 - y21;
    B2 = x21 - x22;
    C2 = A2 * x21 + B2 * y22;

    denominator = A1 * B2 - A2 * B1;
    if denominator <= 0.00001 %平行线
        disp('平行线')
        p = NaN;
        return;
    end

    x = (B2 * C1 - B1 * C2);
    y = (A1 * C2 - A2 * C1);

    p = [x, y];
end

function x = is_in_2d_polygon

    global ptCloud step_len sp tp pointUser pointUserNext pointUserPrev;

    x = false;

    px = pointUser(1);
    py = pointUser(2);

    angle_sum = 0;

    dis = 0;
    dis_min = 10000;

    j = size(ptCloud.Location, 1);

    for i = 1:size(ptCloud.Location, 1)
        sx = ptCloud.Location(i, 1);
        sy = ptCloud.Location(i, 2);
        tx = ptCloud.Location(j, 1);
        ty = ptCloud.Location(j, 2);

        A = sy - ty;
        B = tx - sx;
        C = (sx-tx)*ty-(sy-ty)*tx;

        ab = A^2 + B^2;

        if ab >= 0.002^2
            
            dis = abs(A*px + B*py + C) / sqrt(ab);
            
            if dis < 0.002
                if (px >= sx && px <= tx) || (px <= sx && px >= tx)
                 x = true;
                 return;
                end
            else
                if dis < dis_min
                    dis_min = dis;
                end
            end

            angle = atan2(sy-py, sx-px)-atan2(ty-py,tx-px);
            if angle >= pi
                angle = angle - pi * 2;
            elseif angle <= -pi
                angle = angle + pi * 2;
            end

            angle_sum = angle_sum + angle;
            j = i;

        end

    end

    x = abs(angle_sum - pi * 2) < 1e-4;
    return;

end