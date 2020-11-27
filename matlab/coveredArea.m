function A = coveredArea(fileName)
    warning('off','all')
    disp("read data")
    tic
    fileData = importdata(fileName);
    toc
%     data = fileData.data(1:1000,:);
    data = fileData.data;
    data = data(3:size(data,1),:);
    P = data(:,1:3);
    t = data(:,4);
    mat = data(:,5:13);
    
    disp("get flat path")
    tic
    TR = widthPath(P,0.5,mat);
    toc
    
    disp("Area")
    tic
    A = pathArea(TR,mat);
    toc