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
    2 1 0];

plot(points(:,1),points(:,2))
axis equal

TR = widthPath(points,.5);
figure;
% plot(TR(:,1),TR(:,2),'or');
% axis equal
trimesh(TR,'FaceColor','none','EdgeColor','k')
axis equal

figure;
hold on
p1=[0 0 0;
    1 0 0;
    0 0 0;
    0 1 0;
    0 0 0;
    0 0 1];
plotIt(p1)

euler = [0 0 0];
p2 = addIt(rotateE(euler,p1,0),[1 1 1]);
plotIt(p2)

p3 = addIt(rotateE(euler,p2,1),[1 1 1]);
% p3 = addIt(rotateE(euler,addIt(p2,-[1 1 1]),1),[2 2 2]);
plotIt(p3)

xlabel('x')
ylabel('y')
zlabel('z')
axis equal
hold off

function plotIt(p)
plot3(p(1:2,1),p(1:2,2),p(1:2,3),'g')
plot3(p(3:4,1),p(3:4,2),p(3:4,3),'r')
plot3(p(5:6,1),p(5:6,2),p(5:6,3),'b')
end

function addPoints = addIt(points,p)
addPoints = zeros(size(points));
for i = 1:size(points,1)
    addPoints(i,:) = points(i,:) + p;
end
end