clear;clc;close all;

x = 1:24*60;
y = log(x)/2/15*100;
plot(x,y,'LineWidth',4)
hold on

y = log(x)/15*100;
plot(x,y,'LineWidth',4)

y = 1.5*log(x)/15*100;
plot(x,y,'LineWidth',4)

y = 2*log(x)/15*100;
plot(x,y,'LineWidth',4)

legend('Random Walk','Augmented Random Walk','BEACON Based Algorithm','Full Coverage','FontSize',12)
xlabel('time[min]','FontSize',16)
ylabel('coverage[%]','FontSize',16)