clear;clc;close all;
warning('off','all')
[A,t,O] = coveredArea("C:\Data\Aalborg\Project\FILES\ran1_1\ran1.1\visited_point1.txt");

disp("REMEMBER TO SAVE DATA")
save("data_DEFAULT.mat")