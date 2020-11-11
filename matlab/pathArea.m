function area = pathArea(TR,euler)
P = TR.Points; % get points from the triangulation
T = TR.ConnectivityList; % get triangles from the triangulation

area = zeros(1,size(T,1)/2); % will be the accumulated area on each state

for i = 1:size(T,1)/2 % go through all the sections of the path
    tri = T(2*i-1:2*i,:); % get the triangles of the section
    rect1 = getRect(tri); % create the rectangle of the current section with the triangles
    
    if i == 1 % on the first iteration
        for j = 1:2
            triangle = P(tri(j,:),:);
            preA = triangArea(triangle); % get the area
        end
        area(i) = preA; % and put it on the first slot of the area
    else
        eulIndex = max(max(tri));
        eulIndex = int16(eulIndex/2);
        eu = euler(eulIndex,:); % get the angles to use
        PR = rotateE(eu,P(1:2*(i+1),:),0); % rotate the points
        po1 = PR(rect1,:); % get points of the current quadilateral
        poly1 = polyshape(po1(:,1),po1(:,2)); % create quadrilateral
        [av1,rng1] = averageZ(po1); % get averages
        j = 1;
        while j < i % for all the sections before the actual one
            rect2 = T(2*j-1:2*j,:); % geth a previous one
            rect2 = getRect(rect2);
            po2 = PR(rect2,:);
            [av2,rng2] = averageZ(po2); % get averages
            if abs(rng1-rng2) < .5 && abs(av1-av2) < .5 % if polygons are more or lees on the similar plane
                poly2 = polyshape(po2(:,1),po2(:,2)); % create second polygon
                poly1 = subtract(poly1,poly2); % subtract to the one looking
            end
            if poly1.area == 0 % if the sections checked is completly overlapped
                j = i; % end iteration
            else
               j = j + 1; 
            end
        end
        area(i) = area(i-1) + poly1.area; % next accumulated area
    end
end