function line = getLine(p1,p2)
line = polyfit([p1(1) p2(1)],[p1(2) p2(2)],1);