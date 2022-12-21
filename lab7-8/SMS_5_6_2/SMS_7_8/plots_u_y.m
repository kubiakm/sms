%% Wyjście
yfilename = "y_zg";

yfilepath = append("figs/", yfilename, ".fig");
ysavepath = append("pngs/", yfilename, ".png");

fig = openfig(yfilepath, "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

t = dataObjs(1).XData;
y = dataObjs(1).YData;

kp = 1;
len = 200; 
t = t(1:len);
y = y(kp:len+kp-1);

%% Sterowanie
ufilename = "u_zg";
ufilepath = append("figs/", ufilename, ".fig");
usavepath = append("pngs/", ufilename, ".png");

fig = openfig(ufilepath, "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

u = dataObjs(1).YData;
u = u(kp:len+kp-1);

%% Wykresy
% Wyjście
figure(10)
plot(t, y, 'Color', 'Black');
hold on
title('Wyjście:'); xlabel('t'); ylabel('y');
xlim([0, t(length(t))]);
print(ysavepath,'-dpng','-r300')
close all

% Sterowanie
figure(11)
plot(t, u, 'Color', 'Black');
title('Sterowanie:'); xlabel('t'); ylabel('u');
xlim([0, t(length(t))]);
print(usavepath,'-dpng','-r300')
close all
