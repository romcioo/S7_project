clear;clc;close all;

% % elem = [1 2 5;
% %     2 3 5;
% %     1 4 5;
% %     3 5 6;
% %     4 5 7;
% %     5 6 9;
% %     5 7 8;
% %     5 8 9];
% % 
% % nodes = [0 0;
% %     8 0;
% %     17 0;
% %     0 7;
% %     8 7;
% %     17 7;
% %     0 15;
% %     8 15;
% %     17 15];
% % 
% % nodes = [nodes zeros(size(nodes,1),1)];
% % 
% % p1 = [1 1 0];
% % 
% % TR = triangulation(elem,nodes);
% % trimesh(TR,'FaceColor','none','EdgeColor','k')
% % axis equal;
% % hold on
% % plot3(p1(1),p1(2),p1(3),'or')
% 
% % points = [0 0 0;
% %     1 0 0;
% %     2 1 0;
% %     3 2 1];
% 
% points = [0 0 0;
%     0 1 0];
% 
% plot3(points(:,1),points(:,2),points(:,3))
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% % eu = zeros(size(points));
% % eu(:,3) = pi/3*ones(size(points,1),1);
% % eu = [0 0 0;
% %     0 0 0;
% %     0 0 pi/4;
% %     0 -pi/4 pi/4];
% 
% eu = [0 0 0;
%     0 0 pi/4];
% 
% TR = widthPath(points,.5,eu);
% figure;
% % plot(TR(:,1),TR(:,2),'or');
% % axis equal
% trimesh(TR,'FaceColor','none','EdgeColor','k')
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% 
% % A = triangArea(TR);
% 
% % figure;
% % hold on
% % p1=[0 0 0;
% %     1 0 0;
% %     0 0 0;
% %     0 1 0;
% %     0 0 0;
% %     0 0 1];
% % plotIt(p1)
% % 
% % euler = [0 0 0];
% % p2 = addIt(rotateE(euler,p1,0),[1 1 1]);
% % plotIt(p2)
% % 
% % p3 = addIt(rotateE(euler,p2,1),[1 1 1]);
% % % p3 = addIt(rotateE(euler,addIt(p2,-[1 1 1]),1),[2 2 2]);
% % plotIt(p3)
% % 
% % xlabel('x')
% % ylabel('y')
% % zlabel('z')
% % axis equal
% % hold off
% 
% % function plotIt(p)
% % plot3(p(1:2,1),p(1:2,2),p(1:2,3),'g')
% % plot3(p(3:4,1),p(3:4,2),p(3:4,3),'r')
% % plot3(p(5:6,1),p(5:6,2),p(5:6,3),'b')
% % end
% % 
% % function addPoints = addIt(points,p)
% % addPoints = zeros(size(points));
% % for i = 1:size(points,1)
% %     addPoints(i,:) = points(i,:) + p;
% % end
% % end

% figure
% p = [1 0 0;0 0 0;0 1 0;1 1 0];
% num = [1 2;3 4];
% T = getTriangulation(p,num);
% TR = triangulation(T,p);
% trimesh(TR,'FaceColor','none','EdgeColor','k')
% rect = getRect(T);
% rect1 = p(rect,:);
% % figure
% % hold on
% % plot3(p(rect(1),1),p(rect(1),2),p(rect(1),3),'or');
% % plot3(p(rect(2),1),p(rect(2),2),p(rect(2),3),'og');
% % plot3(p(rect(3),1),p(rect(3),2),p(rect(3),3),'ob');
% % plot3(p(rect(4),1),p(rect(4),2),p(rect(4),3),'ok');
% rect2 = [.5 .5 0;2 .5 0;.5 2 0;2 2 0];
% T = getTriangulation(rect2,num);
% TR1 = triangulation(T,rect2);
% rect22 = getRect(TR1.ConnectivityList);
% rect2 = rect2(rect22,:);
% hold on
% trimesh(TR1,'FaceColor','none','EdgeColor','g')
% [in,on] = inpolygon(rect1(:,1),rect1(:,2),rect2(:,1),rect2(:,2));
% [a,r] = averageZ(rect1);
% poly1 = polyshape(rect1(:,1),rect1(:,2));
% poly2 = polyshape(rect2(:,1),rect2(:,2));
% polyout = intersect(poly1,poly2);
% plot(polyout)
% area = polyout.area;

p1 = [1 0;2 0;2 2;1 2];
poly1 = polyshape(p1(:,1),p1(:,2));

p2 = [0 .5;3 .5;3 1.5;0 1.5];
poly2 = polyshape(p2(:,1),p2(:,2));

plot(poly1)
hold on
plot(poly2)
axis equal

figure
rest = intersect(poly1,poly2);
poly3 = subtract(poly1,rest);
plot(poly3)
axis equal

figure
poly4 = subtract(poly1,poly2);
plot(poly4)
axis equal
title('Bona')

p5 = [1 0;2 0;2 .5;1 .5];
poly5 = polyshape(p5(:,1),p5(:,2));
poly6 = subtract(poly4,poly5);
figure
plot(poly6)
axis equal

figure
poly7 = subtract(poly4,poly1);
plot(poly7)
axis equal