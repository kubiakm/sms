delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
s = serialport('COM3', 115200, 'Parity', 'None');
s.configureTerminator("LF");
fopen(s); % otwarcie kanalu komunikacyjnego

% Tp = 0.1; % czas z jakim probkuje regulator
% y = []; % wektor wyjsc obiektu
% u = []; % wektor wejsc (sterowan) obiektu
% k = 1;
% kk = 20;

while true
    txt = s.readline();
    eval(char(txt')); % wykonajmy to co otrzymalismy
    fprintf('y: %f\n', Y);
end