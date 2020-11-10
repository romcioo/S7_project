function evaluationFunction(file)
    filedata = importdata(file); % get the info from the .txt
    filename = filedata.textdata{1,1}; % get the .stl file name
    fileNum = filedata.data(2,1); % get the number of the evaluation
    fig = figure; % create a figure
    TR = stlread(filename); % read the .stl file and create a triangulation
    TR = rotate(TR); % rotate the TR
    trimesh(TR,'FaceColor','none','EdgeColor','k') % plot the TR with BLACK edges and NO faces
    axis equal % make the axis have the same scaling
    %  [Have the axis labels]
    xlabel('x')
    ylabel('y')
    zlabel('z')
    %  [!Have the axis labels!]
    hold on % keep open the plot to be able to plot on top of it
    % [Draw the points where the robot has gone through]
    for i = 3:size(filedata.data,1) % change to 3 if scale in file
        p = filedata.data(i,:); % get point
        plot3(p(1),p(2),p(3),'or') % plot the point in red circles
    end
    % [!Draw the points where the robot has gone through!]
    hold off % stop to hold the plot
    fn = sprintf("dedBasil%d.fig",fileNum); % create the name file
    saveas(fig,fn); % save the figure
end

% [Rotate the plot]
function TR = rotate(read)
    coords = read.Points*1e-3; % convert from mm to m
    orderList = order(coords);
    P = [coords(:,orderList(1)) coords(:,orderList(2)) coords(:,orderList(3))];
    P = center(P);
    T = read.ConnectivityList;
    T = [T(:,orderList(1)) T(:,orderList(2)) T(:,orderList(3))];
    TR = triangulation(T,P);
end
% [!Rotate the plot!]

% [Get the order to change]
function orderList = order(coords)
    orderList = [0 0 0]; % x y z
    count = 0; % index count
    maxim = 0; % max value
    minim = 0; % min value
    for i = 1:2
        index = 0; % index value
        for j = 1:3
            m = max(coords(:,j)); % get the maximum value of the 
            if i == 1 % if its the first iteration
                if m > maxim % check for the maximum length
                    maxim = m; % set the maximum value found
                    minim = m; % set the minimum value to the maximum to set it up for the next iteration
                    index = j; % get the coordenate that satisfies it
                end
            else % second iteration
                if m < minim % look for the lowest value
                    minim = m; % set the min value
                    index = j; % get the index
                end
            end
        end
        if i == 1 % set the long length
            orderList(1) = index; % as the x
        else % set the shortest
            orderList(3) = index; % to the z coordinates
        end
        count = count + index; % sum indexes
    end
    if count == 3 % if the indexes used are 1 and 2
        index = 3; % last index is 3
    elseif count == 4 % if index are 1 and 3
        index = 2;
    else % if index 2 and 3
        index = 1;
    end
    orderList(2) = index; % set last index
end
% [!Get the order to change!]

% [Offset]
function centered = center(P)
    centered = P;
%     for i = 1:2
%         maxim = max(P(:,i));
%         offset = -maxim/2;
%         centered(:,i) = P(:,i) + offset*ones(size(P(:,i)));
%     end
    offset = -max(P(:,3)); % set the top on the water line
    i = 3;
    centered(:,i) = P(:,i) + offset*ones(size(P(:,i)));
end
% [!Offset!]