function coveredArea(fileName1,fileName2)
    warning('off','all')
    disp("read data")
    tic
    fileData = importdata(fileName1);
    matData = importdata(fileName2);
    toc
%     data = fileData.data(1:1000,:);
    data = fileData.data;
    data = data(3:size(data,1),:);
    P = data(:,1:3);
    t = data(:,4);
%     mat = data(:,5:13);
    area = data(:,14);
    overlap = data(:,15);
%     realP = data(:,16:18);
    sizeMatrix = matData(1,:);
    matrixData = matData(2:size(matData,1),:);
    
    figure;
    plot(t,area)
    title('Accummulated Area')
    ylim([0,(max(area)+.25)])
    xlabel('time [s]')
    ylabel('Area [m^2]')
    
    figure
    plot(t,overlap)
    title('Overlapment')
    ylim([0,(max(overlap)+.25)])
    xlabel('time [s]')
    ylabel('Area [m^2]')