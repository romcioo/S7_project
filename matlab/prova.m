clear;clc;close all;
warning('off','all')

% tic
% points = [0 0 0;
%     1 0 0;
%     -1 0 0;
%     0 0 0;
%     0 1 0;
%     0 -1 0;
%     0 1 0];
% % points = [1 0 0;-1 0 0];
% 
% % plot3(points(:,1),points(:,2),points(:,3))
% % axis equal
% % xlabel('x')
% % ylabel('y')
% % zlabel('z')
% 
% eu = [0 0 0;
%     0 0 0;
%     0 0 pi;
%     0 0 0;
%     0 0 pi/2;
%     0 0 -pi/2;
%     0 0 pi/2];
% % eu = [0 0 0; 0 0 pi];
% TR = widthPath(points,.5,eu);
% % figure;
% % trimesh(TR,'FaceColor','r','EdgeColor','k')
% % axis equal
% % xlabel('x')
% % ylabel('y')
% % zlabel('z')
% 
% A = pathArea(TR,eu);
% toc

a = zeros([1,2e6]);
parfor i = 1:2e6
    a(i) = 2*i;
end