%% Wyjście
yfilename = "pid_inz_rozn_test_y_Td_005";

yfilepath = append("figs/", yfilename, ".fig");
ysavepath = append("pngs/", yfilename, ".png");

fig = openfig(yfilepath, "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

t = dataObjs(1).XData;
yzad = dataObjs(1).YData;
y = dataObjs(2).YData;

kp = find(yzad ~= 0, 1) - 10;
len = 120; 
t = t(1:len);
y = y(kp:len+kp-1);
yzad = yzad(kp:len+kp-1);

%% Sterowanie
ufilename = "pid_inz_rozn_test_u_Td_005";
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
stairs(t, y, 'Color', 'Black');
hold on
stairs(t, yzad, '--r');
title('Wyjście:'); xlabel('t'); ylabel('y');
legend("y", 'yzad','Location', 'southeast'); legend('boxoff');
xlim([0, t(length(t))]);
print(ysavepath,'-dpng','-r300')
close all

% Sterowanie
figure(11)
stairs(t, u, 'Color', 'Black');
title('Sterowanie:'); xlabel('t'); ylabel('u');
xlim([0, t(length(t))]);
print(usavepath,'-dpng','-r300')
close all

%% Błąd
E_val = sum((yzad - y).^2)

