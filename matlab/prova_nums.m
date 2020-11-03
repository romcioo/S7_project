clear;clc;close all;

% elem = [1 2 5;
%     2 3 5;
%     1 4 5;
%     3 5 6;
%     4 5 7;
%     5 6 9;
%     5 7 8;
%     5 8 9];
% 
% nodes = [0 0;
%     8 0;
%     17 0;
%     0 7;
%     8 7;
%     17 7;
%     0 15;
%     8 15;
%     17 15];
% 
% nodes = [nodes zeros(size(nodes,1),1)];
% 
% p1 = [1 1 0];
% 
% TR = triangulation(elem,nodes);
% trimesh(TR,'FaceColor','none','EdgeColor','k')
% axis equal;
% hold on
% plot3(p1(1),p1(2),p1(3),'or')

points = [0 0 0;
    0 1 0;
    1 2 0;
    -1 -1 0];

plot(points(:,1),points(:,2))
axis equal

TR = widthPath(points,.5);
figure;
% plot(TR(:,1),TR(:,2),'or');
% axis equal
trimesh(TR,'FaceColor','none','EdgeColor','k')
axis equal