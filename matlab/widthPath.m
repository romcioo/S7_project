function TR = widthPath(points,width,eu)
shift = width/2;
rP = [0 -1 0;1 0 0;0 0 1];
rN = [0 1 0;-1 0 0;0 0 1];
P = zeros(2*size(points,1),size(points,2));
T = zeros(size(points,1),3);
for i = 2:size(points,1)
    p1 = points(i,:);
    p2 = points(i-1,:);
    vect = p1 - p2;
    vect = rotateE(eu(i,:),vect,0);
    angle = atan(vect(2)/vect(1));
    shiftD = shift*[cos(angle) sin(angle) 0];
    pT = zeros(4,3);
    eui = eu(i,:);
    if i == 2
        P(1,:) = rotateE(eui,(rP*shiftD')',1) + p2;
        P(2,:) = rotateE(eui,(rN*shiftD')',1) + p2;
    end
    P(2*i-1,:) = rotateE(eui,(rP*shiftD')',1) + p1;
    P(2*i,:) = rotateE(eui,(rN*shiftD')',1) + p1;
    n = 2*(i-1);
    for j = 1:4
        pT(j,:) = rotateE(eui,P(2*i-(4-j),:),0);
    end
    nums = [n-1 n;n+1 n+2];
    Trings = getTriangulation(pT,nums);
    T(n-1,:) = Trings(1,:);
    T(n,:) = Trings(2,:);
end
TR = triangulation(T,P);