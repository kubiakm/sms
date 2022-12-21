delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
s = serialport('COM3', 115200, 'Parity', 'None');
s.configureTerminator("LF");
fopen(s); % otwarcie kanalu komunikacyjnego

Tp = 0.1; % czas z jakim probkuje regulator
y = []; % wektor wyjsc obiektu
u = []; % wektor wejsc (sterowan) obiektu
yzad = [];
k = 1;
kk = 300;

while k ~= kk
    txt = s.readline();
    eval(char(txt')); % wykonajmy to co otrzymalismy
    fprintf('k: %i; u: %f; y: %f\n', k, U, Y);
    if U ~= 0
      y=[y;Y]; % powiekszamy wektor y o element Y
      yzad = [yzad; Yzad];
      u=[u;U]; % powiekszamy wektor u o element U
      k = k + 1;
    end
end

filename = "dmc_figs/dmc_N_%d_Nu_%d_lamb_%s";
filename = sprintf(filename, 30, 30, '05');

figure(1);
plot((0:(length(y)-1))*Tp,y); % wyswietlamy y w czasie
hold on
plot((0:(length(yzad)-1))*Tp,yzad); % wyswietlamy y w czasie
hold off
title("y");
y_filename = strcat(filename, "_y", ".fig");
savefig(y_filename);

figure(2);
plot((0:(length(u)-1))*Tp,u); % wyswietlamy u w czasie
title("u");
u_filename = strcat(filename, "_u", suffix, ".fig");
savefig(u_filename);

clear s
