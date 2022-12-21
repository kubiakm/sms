clear all
%% Pliki
zg_filename = "pid_zg_test_1_y";
inz_filename = "pid_inz_rozn_test_y_Td_01";

zg_filepath = append("figs/", zg_filename, ".fig");
inz_filepath = append("figs/", inz_filename, ".fig");

savepath = append("pngs/", "pid_zg_vs_inz", ".png");

%% zg
fig = openfig(zg_filepath, "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

t = dataObjs(1).XData;
yzad = dataObjs(1).YData;
yzg = dataObjs(2).YData;

kp = find(yzad ~= 0, 1) - 10;
len = 120; 

t = t(1:len);
yzg = yzg(kp:len+kp-1);
yzad = yzad(kp:len+kp-1);

%% inz
fig = openfig(inz_filepath, "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

t = dataObjs(1).XData;
yzad = dataObjs(1).YData;
yinz = dataObjs(2).YData;

kp = find(yzad ~= 0, 1) - 10;
len = 120; 
t = t(1:len);
yinz = yinz(kp:len+kp-1);
yzad = yzad(kp:len+kp-1);

%% Wykresy
figure(10)
stairs(t, yinz, 'Color', 'Green');
hold on
stairs(t, yzg, 'Color', 'Blue');
stairs(t, yzad, '--r');
title('Wyjście:'); xlabel('t'); ylabel('y');
legend("y metoda inżynierska", "y Ziegler-Nichols", 'yzad','Location', 'southeast'); legend('boxoff');
xlim([0, t(length(t))]);
print(savepath,'-dpng','-r300')
close all

%% Błąd
E_zg = sum((yzad - yzg).^2)
E_inz = sum((yzad - yinz).^2)

