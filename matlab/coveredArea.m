function A = coveredArea(fileName)
    fileData = importdata(fileName);
    data = fileData.data;
    data = data(3:size(data,1),:);
    P = data(1:3,:);
    t = data(4,:);
    mat = data(5:13,:);
    
    TR = widthPath(P,0.5,mat);
    
    A = pathArea(TR,mat);