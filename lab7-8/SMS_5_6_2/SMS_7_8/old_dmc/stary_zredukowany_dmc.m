%% Oszczędna wersja algorytmu DMC.
%% Warunki początkowe:
u(1:kk) = 0; y(1:kk) = 0; e(1:kk)=0;
yzad(1:stepTick-1) = 0; yzad(stepTick:kk) = stepValue;
%% Macierz M:
M = zeros(N, Nu);
for j = 1:Nu
    for i = j:N
        if i-j+1 > D
            M(i,j) = S(D);
        else
            M(i,j) = S(i-j+1);
        end
    end
end
%% Macierz lambda: NuxNu
LBD = lambda*eye(Nu);
%% Macierz K: NuxN
K = inv(M'*M+LBD)*M';
%% Macierz MP: Nx(D-1)
MP = zeros(N, D-1);
for i = 1:N
    for j = 1:D-1
        first = 0; second = 0;
        if i+j > D
            MP(i, j) = S(D) - S(j);
        else
            MP(i, j) = S(i+j) - S(j);
        end
    end
end
%% Do wyliczenia du:
Kju = K(1, :)*MP;
%% Badanie obszarów stabilności:
% alfa = 2.005; dodatOp = 0; kp = kp + dodatOp;
%% Główna pętla algorytmu:
for k = kp:kk
    % Wyjście modelu:
    y(k) = alfa*(b1*u(k-1-op-dodatOp)+b0*u(k-2-op-dodatOp))-a1*y(k-1)-a0*y(k-2);
    % Uchyb:
    e(k) = yzad(k) - y(k);
    % Zmiana sterowania:
    du = 0;
    for p = 1:N
        du = du + K(1, p)*e(k); % yzad stałe
    end
    for j = 1:D-1
        if k-j < 1
            du = du - Kju(j)*0;
        elseif k-j-1 < 1
            du = du - Kju(j)*(u(k-j)-0);
        else
            du = du - Kju(j)*(u(k-j) - u(k-j-1));
        end
    end
    % Sterowanie = sterowanie z poprzedniej sekundy + oblicz. zmiana sterowania 
    u(k) = u(k-1) + du;
    % Ograniczenie sterowania:
%     if u(k) > umax
%         u(k) = umax;
%     elseif u(k) < -umax
%         u(k) = -umax;
%     end
end
