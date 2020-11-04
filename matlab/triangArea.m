function area = triangArea(triangle)
area = 0;
ones = [1 1 1];

x = triangle(:,1)';
y = triangle(:,2)';
z = triangle(:,3)';
area = area + 0.5*sqrt(det([x;y;ones])^2 + det([y;z;ones])^2 + det([z;x;ones])^2);