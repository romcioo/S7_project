function area = pathArea(TR,euler)
P = TR.Points;
T = TR.ConnectivityList;

area = 0;

for i = 1:size(T,1)/2
    preA = 0;
    tri = T(2*i-1:2*i,:);
    for j = 1:2
        triangle = P(tri(1,:),:);
        preA = preA + triangArea(triangle);
    end
    
    if i == 1
        area = preA;
    else
        eulIndex = max(max(tri));
        eulIndex = int16(eulIndex/2);
        eu = euler(eulIndex,:);
        for j = 1:i-1
            PR = rotateE(eu,P,0);
            
        end
    end
end