function inter = quadIntersect(R1,R2,P,EU)
eu = repmat(EU,size(P,1),1);
P = rotateE(eu,P,0);
r1 = P(R1,:);
[av1,rng1] = averageZ(r1);
r2 = P(R2,:);
[av2,rng2] = averageZ(r2);
if abs(rng1-rng2) > .5
    inter = 0;
elseif abs(av1-av2) > .5
    inter = 0;
else
    poly1 = polyshape(r1(:,1),r1(:,2));
    poly2 = polyshape(r2(:,1),r2(:,2));
    inter = intersect(poly1,poly2);
    inter = inter.area;
end