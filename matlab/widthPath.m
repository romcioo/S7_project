function TR = widthPath(points,width,eu)
shift = width/2; % displacement to each side
rP = [0 -1 0;1 0 0;0 0 1]; % 90º rotation matrix
rN = [0 1 0;-1 0 0;0 0 1]; % -90º rotation matrix
P = zeros(2*size(points,1),size(points,2));
T = zeros(size(points,1),3);
for i = 2:size(points,1) % go through all points
    p1 = points(i,:); % point to look at
    p2 = points(i-1,:); % point behind it
    vect = p1 - p2; % vector p2-p1
    vect = rotateE(eu(i,:),vect,0); % put it to robots plane
    angle = atan(vect(2)/vect(1)); % get angle on x-y plane
    shiftD = shift*[cos(angle) sin(angle) 0]; % get the vector direction coordinates to make the path width
    pT = zeros(4,3);
    eui = eu(i,:); % get the angles of the points
    if i == 2 % for the first iteration
        P(1,:) = rotateE(eui,(rP*shiftD')',1) + p2; % get the 2 firts points in global coordinates
        P(2,:) = rotateE(eui,(rN*shiftD')',1) + p2;
    end
    P(2*i-1,:) = rotateE(eui,(rP*shiftD')',1) + p1; % points in global coordinates
    P(2*i,:) = rotateE(eui,(rN*shiftD')',1) + p1;
    n = 2*(i-1);
    for j = 1:4
        pT(j,:) = rotateE(eui,P(2*i-(4-j),:),0); % get the points on the robot coordinates
    end
    nums = [n-1 n;n+1 n+2];
    Trings = getTriangulation(pT,nums); % get the trinagles
    T(n-1,:) = Trings(1,:);
    T(n,:) = Trings(2,:);
end
TR = triangulation(T,P);