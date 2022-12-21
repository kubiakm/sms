delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
s = serialport('COM5', 115200, 'Parity', 'None');
s.configureTerminator("LF");
fopen(s); % otwarcie kanalu komunikacyjnego

Tp = 0.1; % czas z jakim probkuje regulator
y = []; % wektor wyjsc obiektu
u = []; % wektor wejsc (sterowan) obiektu
k = 1;
kk = 300;

while k ~= kk
    txt = s.readline();
    eval(char(txt')); % wykonajmy to co otrzymalismy
    if U ~= 0
      y=[y;Y]; % powiekszamy wektor y o element Y
      u=[u;U]; % powiekszamy wektor u o element U
      k = k + 1;
    end
end
figure(1);
plot((0:(length(y)-1))*Tp,y); % wyswietlamy y w czasie
title("y");
savefig("y_zg.fig");

figure(2);
plot((0:(length(u)-1))*Tp,u); % wyswietlamy u w czasie
title("u");
savefig("u_zg.fig");

clear s

Ku = 16.5
Tu = (12.9 - 7.5) / 10