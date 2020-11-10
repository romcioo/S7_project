function evaluationFunction(file)
    filedata = importdata(file);
    filename = filedata.textdata{1,1};
    fileNum = filedata.data(2,1);
    fig = figure;
    TR = stlread(filename);
    TR = rotate(TR);
    assignin('base','TR',TR);
    trimesh(TR,'FaceColor','none','EdgeColor','k')
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on
    for i = 3:size(filedata.data,1) % change to 3 if scale in file
        p = filedata.data(i,:);
        plot3(p(1),p(2),p(3),'or')
    end
    hold off
    fn = sprintf("dedBasil%d.fig",fileNum);
    saveas(fig,fn);
end

function TR = rotate(read)
    coords = read.Points*1e-3; % from mm to m
    orderList = order(coords);
    P = [coords(:,orderList(1)) coords(:,orderList(2)) coords(:,orderList(3))];
    P = center(P);
    T = read.ConnectivityList;
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