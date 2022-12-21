delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
s = serialport('COM3', 115200, 'Parity', 'None');
s.configureTerminator("LF");
fopen(s); % otwarcie kanalu komunikacyjnego

Tp = 1; % czas z jakim probkuje regulator
y = []; % wektor wyjsc obiektu
u = []; % wektor wejsc (sterowan) obiektu
k = 1;
kk = 300;

% Punkt pracy: 37.6

while k ~= kk
    txt = s.readline();
    eval(char(txt')); % wykonajmy to co otrzymalismy
    fprintf('k: %i\n', k);
    if U ~= 0
      y=[y;Y]; % powiekszamy wektor y o element Y
      u=[u;U]; % powiekszamy wektor u o element U
      k = k + 1;
    end
end
figure(1);
plot((0:(length(y)-1))*Tp,y); % wyswietlamy y w czasie
title("y");
savefig("s_y.fig");

figure(2);
plot((0:(length(u)-1))*Tp,u); % wyswietlamy u w czasie
title("u")
savefig("s_u.fig");
%% 
s=(y(2:kk-1)-y(1))/20.0; % przeskalowane pomiary = jednostkowa odpowiedz skokowa
figure(3);
plot((0:(length(s)-1))*Tp,s); % wyswietlamy u w czasie
title("s");
savefig('s.fig')

%%

