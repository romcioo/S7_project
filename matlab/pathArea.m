function area = pathArea(TR,euler)
P = TR.Points;
T = TR.ConnectivityList;

area = zeros(1,size(T,1)/2);

for i = 1:size(T,1)/2
    tri = T(2*i-1:2*i,:);
    rect1 = getRect(tri);
    for j = 1:2
        triangle = P(tri(j,:),:);
        preA = triangArea(triangle);
    end
    
    if i == 1
        area(i) = preA;
    else
        eulIndex = max(max(tri));
        eulIndex = int16(eulIndex/2);
        eu = euler(eulIndex,:);
        PR = rotateE(eu,P,0);
        po1 = PR(rect1,:);
        poly1 = polyshape(po1(:,1),po1(:,2));
        [av1,rng1] = averageZ(po1);
        j = 1;
        while j < i
            rect2 = T(2*j-1:2*j,:);
            rect2 = getRect(rect2);
            po2 = PR(rect2,:);
            poly2 = polyshape(po2(:,1),po2(:,2));
            [av2,rng2] = averageZ(po2);
            if abs(rng1-rng2) < .5 && abs(av1-av2) < .5
                poly1 = subtract(poly1,poly2);
            end
            if poly1.area == 0
                j = i;
            else
               j = j + 1; 
            end
        end
        area(i) = area(i-1) + poly1.area;
    end
end