fig = openfig('s.fig', "invisible");
axObjs = fig.Children;
dataObjs = axObjs.Children;

k = dataObjs(1).XData;
s = dataObjs(1).YData;
kk = length(s);

figure(3);
plot((0:(length(s)-1))*Tp,s); % wyswietlamy u w czasie

x = 5;
for i = x+1:kk-(x+1)
    s(i) = sum(s(i-x:i+x))/(2*x+1);
end
figure(4);
plot((0:(length(s)-1))*Tp,s); % wyswietlamy u w czasie
title("s");
savefig('s_avg.fig')

save('S', 's');


% clear s