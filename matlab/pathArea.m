function [areaAcum,time,overlapAcum] = pathArea(TR,mat,t)
P = TR.Points; % get points from the triangulation
T = TR.ConnectivityList; % get triangles from the triangulation

area = zeros(1,size(T,1)/2);
areaAcum = zeros(1,size(T,1)/2); % will be the accumulated area on each state
overlap = zeros(1,size(T,1)/2);
overlapAcum = zeros(1,size(T,1)/2);

parfor i = 1:size(T,1)/2 % go through all the sections of the path
    tri = T(2*i-1:2*i,:); % get the triangles of the section
    rect1 = getRect(tri); % create the rectangle of the current section with the triangles
    
    if i == 1 % on the first iteration
        preA = 0;
        for j = 1:2
            triangle = P(tri(j,:),:);
            preA = preA + triangArea(triangle); % get the area
        end
        area(i) = preA; % and put it on the first slot of the area
    else
        matIndex = max(max(tri));
        matIndex = int16(matIndex/2);
        rotation = mat(matIndex,:); % get the angles to use
        rotation = reshape(rotation,[3,3]);
        PR = rotateE(rotation,P(1:2*(i+1),:),0); % rotate the points
        po1 = PR(rect1,:); % get points of the current quadrilateral
        poly1 = polyshape(po1(:,1),po1(:,2)); % create quadrilateral
        initialArea = poly1.area;
        [av1,rng1] = averageZ(po1); % get averages
        for j = 1:(i-1) % for all the sections before the actual one
            rect2 = T(2*j-1:2*j,:); % geth a previous one
            rect2 = getRect(rect2);
            po2 = PR(rect2,:);
            [av2,rng2] = averageZ(po2); % get averages
            if abs(rng1-rng2) < .5 && abs(av1-av2) < .5 % if polygons are more or lees on the similar plane
                poly2 = polyshape(po2(:,1),po2(:,2)); % create second polygon
                poly1 = subtract(poly1,poly2); % subtract to the one looking
            end
            if poly1.area == 0 % if the sections checked is completly overlapped
                break
            end
        end
        finalArea = poly1.area;
        area(i) = finalArea; % next accumulated area
        overlap(i) = (initialArea - finalArea);
    end
end
anterior = 0;
for i = 1:size(areaAcum,1)
    anterior = anterior + area(i);
    areaAcum(i) = anterior;
end
anterior = 0;
for i = 1:size(overlapAcum,1)
    anterior = anterior + overlap(i);
    overlapAcum(i) = anterior;
end
time = t(2:size(t,1),:);