function T = getTriangulation(p,num)
side1 = p(1:2,1:2);
side2 = p(3:4,1:2);
L11 = getLine(side1(1,:),side2(1,:));
L22 = getLine(side1(2,:),side2(2,:));
[~,~,C1] = crossLines(L11,L22,[side1(1,1) side2(1,1);side1(2,1) side2(2,1)]);
if C1
    T = [num(1,1) num(1,2) num(2,1);
        num(1,1) num(2,1) num(2,2)];
else
    T = [num(1,1) num(1,2) num(2,1);
        num(1,2) num(2,1) num(2,2)];
end
    