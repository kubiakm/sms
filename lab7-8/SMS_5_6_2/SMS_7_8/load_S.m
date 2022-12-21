%% Wyjście
clear all
yfilename = "s_y";

yfilepath = append("figs/", yfilename, ".fig");
ysavepath = append("pngs/", yfilename, ".png");

fig = openfig(yfilepath, "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

t = dataObjs(1).XData;
s = dataObjs(1).YData;

s_0 = s(1);
for i=1:length(s)
    s(i) = (s(i)-s_0)/1000;
end
for i=4:length(s)-3
    s(i) = sum(s(i-3:i+3))/7;
end

for sd=2:length(s)
    if abs(s(sd) - s(sd-1)) < 10e-6
        break
    end
end

sd
t2 = t(1:sd);
s_apr = s(1:sd);

figure(10)
plot(t2, s_apr, 'Color', 'Black');
title('Wyjście:'); xlabel('t'); ylabel('S');
xlim([0, t2(length(t2))]);
% print(ysavepath,'-dpng','-r300')
% close all

save('S', 's_apr');
