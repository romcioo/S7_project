function evaluationFunction(file)
%     clearvars fn
    filedata = importdata(file);
%     assignin('base','f',filedata);
%     head = filedata.textdata;
    filename = filedata.textdata{1,1};
%     assignin('base','fN',filename);
    fileNum = filedata.data(2,1);
%     fileNum = filedata.data(1,:);
%     assignin('base','num',fileNum);
%     scale = filedata.data(1,:);
    fig = figure;
    TR = stlread(filename);
    TR = rotate(TR);
    assignin('base','TR',TR);
    scale = max(max(TR.Points))*ones(1,3);
    scaleLine = filedata.data(1,:);
    if scaleLine(4)
        scale = scaleLine(1:3);
    end
    trimesh(TR,'FaceColor','none','EdgeColor','k')
%     axis([-scale(1) scale(1) -scale(2) scale(2) -scale(3) scale(3)]);
%     axis([0 scale(1) 0 scale(2) 0 scale(3)]);
%     axis([0 scale(1) -scale(2)/2 scale(2)/2 -scale(3)/2 scale(3)/2]);
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on
    for i = 3:size(filedata.data,1) % change to 3 if scale in file
        p = filedata.data(i,:);
        plot3(p(1),p(2),p(3),'or')
    end
%     for i = 1:size(TR.ConnectivityList,1)
% %         p = TR.Points(i,:);
% %         plot3(p(1),p(2),p(3),'og')
%         tr = TR.ConnectivityList(i,:);
%         for j = 1:3
%             p = TR.Points(tr(j),:);
%             plot3(p(1),p(2),p(3),'og');
%         end
%         waitforbuttonpress;
%     end
    hold off
    fn = sprintf("dedBasil%d.fig",fileNum);
    saveas(fig,fn);
%     A = triarea(TR.ConnectivityList,TR.Points);
%     assignin('base','A',A);
end

function TR = rotate(read)
    coords = read.Points*1e-3; % from mm to m
    orderList = order(coords);
    P = [coords(:,orderList(1)) coords(:,orderList(2)) coords(:,orderList(3))];
    P = center(P);
    T = read.ConnectivityList;
%     T = removeLid(T,P);
    T = [T(:,orderList(1)) T(:,orderList(2)) T(:,orderList(3))];
    TR = triangulation(T,P);
end

function orderList = order(coords)
    orderList = [0 0 0]; % x y z
    count = 0;
    maxim = 0;
    minim = 0;
    for i = 1:2
        index = 0;
        for j = 1:3
            m = max(coords(:,j));
            if i == 1
                if m > maxim
                    maxim = m;
                    minim = m;
                    index = j;
                end
            else
                if m < minim
                    minim = m;
                    index = j;
                end
            end
        end
        if i == 1
            orderList(1) = index;
%         orderList(i) = index;
        else
            orderList(3) = index;
        end
        count = count + index;
    end
    if count == 3
        index = 3;
    elseif count == 4
        index = 2;
    else
        index = 1;
    end
    orderList(2) = index;
end

function centered = center(P)
    centered = P;
%     for i = 1:2
%         maxim = max(P(:,i));
%         offset = -maxim/2;
%         centered(:,i) = P(:,i) + offset*ones(size(P(:,i)));
%     end
    offset = -max(P(:,3));
    i = 3;
    centered(:,i) = P(:,i) + offset*ones(size(P(:,i)));
end

% function A = triarea(t, p)
% % A = TRIAREA(t, p) area of triangles in triangulation
% Xt = reshape(p(t, 1), size(t)); % X coordinates of vertices in triangulation
% Yt = reshape(p(t, 2), size(t)); % Y coordinates of vertices in triangulation
% A = 0.5 * abs((Xt(:, 2) - Xt(:, 1)) .* (Yt(:, 3) - Yt(:, 1)) - ...
%     (Xt(:, 3) - Xt(:, 1)) .* (Yt(:, 2) - Yt(:, 1)));
% A = sum(A);
% end

% function Tlid = removeLid(T,P)
% top = zeros(1,size(P,1));
% count = 0;
% Tlid = zeros(size(T));
% for i = 1:size(P,1)
%     z = P(i,3);
%     if z == 0
%         count = count + 1;
%         top(count) = i;
%     end
% end
% top = top(1:count);
% count = 0;
% for i = 1:size(T,1)
%     tr = T(i,:);
%     include = 1;
%     for j = 1:3
%         in = size(find(top==tr(j)));
%         if in(2) == 0
%             include = 0;
%         end
%     end
%     if include
%         count = count + 1;
%         Tlid(count,:) = tr;
%     end
% end
% Tlid = Tlid(count,:);
% end