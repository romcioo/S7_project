function line = getLine(p1,p2)
if p1(1) == p2(1)
    line = [NaN p1(1)];
else
    line = polyfit([p1(1) p2(1)],[p1(2) p2(2)],1);
end