%% Formatowanie wykresów
set(0,'defaultLineLineWidth',2);
set(0,'DefaultStairLineWidth',2);
set(0, 'DefaultFigureWindowStyle', 'normal'); % automatycznie wykres w nowym oknie
set(0,'defaultAxesFontSize', 15);
set(0,'defaultfigureposition', [10,10,1000,500]);
%%
yfilename = "y_zg.fig";
ufilename = "u_zg.fig";

set(gcf,'Visible','off')
yfig = openfig(yfilename);
yaxObjs = yfig.Children;
ydataObjs = axObjs.Children;

set(gcf,'Visible','off')
ufig = openfig(ufilename);
uaxObjs = ufig.Children;
udataObjs = axObjs.Children;

%% Y + Yzad
% % t = dataObjs(1).XData;
% % yzad = dataObjs(1).YData;
% % y = dataObjs(2).YData;

%% Y
t = ydataObjs(1).XData;
y = ydataObjs(1).YData;

%% U
t = udataObjs(1).XData;
u = udataObjs(1).YData;

%% 

figure(10)
stairs(t, u);


figure(11)
stairs(t, y);
hold on
stairs(t, yzad);

E_val = sum((yzad - y).^2)