function area = pathArea(TR,euler)
P = TR.Points;
T = TR.ConnectivityList;

area = 0;

for i = 1:size(T,1)/2
    preA = 0;
    tri = T(2*i-1:2*i,:);
    rect1 = getRect(tri);
    for j = 1:2
        triangle = P(tri(j,:),:);
        preA = preA + triangArea(triangle);
    end
    
    if i == 1
        area = preA;
    else
        eulIndex = max(max(tri));
        eulIndex = int16(eulIndex/2);
        eu = euler(eulIndex,:);
        PR = rotateE(eu,P,0);
        for j = 1:i-1
            rect2 = T(2*j-1:2*j,:);
            rect2 = getRect(rect2);
            if j == 1
                [interA,poly1] = quadIntersect(rect1,rect2,PR,0,0);
            else
                [interA,poly1] = quadIntersect(rect1,rect2,PR,poly1,0);
            end
            preA = preA - interA;
        end
        preA = poly1.area;
        area = area + preA;
    end
end