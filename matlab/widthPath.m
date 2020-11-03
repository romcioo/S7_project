function TR = widthPath(points,width)
shift = width/2;
rP = [0 -1 0;1 0 0;0 0 1];
rN = [0 1 0;-1 0 0;0 0 1];
P = zeros(2*size(points,1),size(points,2));
T = zeros(size(points,1),3);
for i = 2:size(points,1)
    p1 = points(i,:);
    p2 = points(i-1,:);
    vect = p1 - p2;
    angle = atan(vect(2)/vect(1));
    shiftD = shift*[cos(angle) sin(angle) 0];
    if i == 2
        P(1,:) = (rP*shiftD')' + p2;
        P(2,:) = (rN*shiftD')' + p2;
    end
    P(2*i-1,:) = (rP*shiftD')' + p1;
    P(2*i,:) = (rN*shiftD')' + p1;
    n = 2*(i-1);
    T(n-1,:) = [n-1 n n+1];
    T(n,:) = [n n+1 n+2];
end
TR = triangulation(T,P);