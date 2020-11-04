function inter = quadIntersect(r1,r2)
inter = 0;
possible = 0;

X1 = r1(:,1);
Y1 = r1(:,2);
X2 = r2(:,1);
Y2 = r2(:,2);

min1 = [min(X1) min(Y1)];
min2 = [min(X2) min(Y2)];
max1 = [max(X1) max(Y1)];
max2 = [max(X2) max(Y2)];

if (((max1(1) > max2(1)) && (max2(1) > min1(1))) || ((max2(1) > max1(1)) && (max1(1) > min2(1))))
    if (((max1(2) > max2(2)) && (max2(2) > min1(2))) || ((max2(2) > max1(2)) && (max1(2) > min2(2))))
        possible = 1;
    end
end