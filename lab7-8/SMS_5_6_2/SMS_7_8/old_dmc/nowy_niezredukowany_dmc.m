clear all;
%Upp =0;
%Ypp = 0;
Upp = 27;
addpath('D:\SerialCommunication'); % add a path to the functions
    initSerialControl COM7 % initialise com port

Ypp = readMeasurements(1);
u_max = 100;
u_min = 0 ;
kk=300;
X = [4.6962   76.8468    0.3253];
T1=X(1);
T2=X(2);
K=X(3);
Td=10;
alpha1 = exp(-1/T1);
    alpha2 = exp(-1/T2);
    a1 = -alpha1 - alpha2;
    a2 = alpha1*alpha2;
    b1 = (K/(T1-T2))*(T1*(1-alpha1)-T2*(1-alpha2));
    b2 = (K/(T1-T2))*(alpha1*T2*(1-alpha2) - alpha2*T1*(1-alpha1));
S(1:600) = 0;
y1(1:length(S)) = 0;
u1(1:length(S)) = 1;

for i=Td+3:length(S)
        y1(i) = b1*u1(i-Td-1) + b2*u1(i-Td-2) - a1*y1(i-1) - a2*y1(i-2);
end
S = y1;

N =40;
Nu = 35;
lamb = 1;
D = 569;
M=zeros(N,Nu);
for i=1:N
    for j=1:Nu
        if(i>=j)
            M(i,j)=S(i-j+1);
        end
    end
end
Da = lamb*eye(Nu);
K = (M'*M+Da)^(-1)*M';

MP = zeros(N,D-1);
for i=1:N
    for j=1:D-1
       
      if i+j<=D
         MP(i,j)=S(i+j)-S(j);
      else
         MP(i,j)=S(D)-S(j);
      end    
    end
end

yzad(1:Td+2)=Ypp;

yzad(Td+2:300)=40;
%yzad(301:kk)=28;
y(1:kk)=Ypp;
u(1:kk)=Upp;
dup = zeros(1,D-1);
for k=Td+3:kk
    %y(k) = b1*u(k-Td-1) + b2*u(k-Td-2) - a1*y(k-1) - a2*y(k-2);
    y(k) = readMeasurements(1);
    for i=1:D-1
        if i+1>=k
           dup(i)=0; 
        else
        dup(i)=u(k-i)-u(k-i-1);
        end
    end
    du=K*(yzad(k)*ones(N,1)-MP*(dup')-y(k)*ones(N,1));
   
    u(k)=u(k-1)+du(1,1);
    if u(k) > u_max
        u(k) = u_max;
    end
    
    if u(k) < u_min
        u(k) = u_min;
    end
     sendControls([ 1, 2, 3, 4, 5, 6], ... send for these elements
                     [50, 0, 0, 0, u(k), 0]);  % new corresponding control values
 waitForNewIteration(); % wait for new batch of measurements to be ready
end

figure(1);
stairs(yzad);
hold on;
stairs(y);
hold off;
print('ytraj1testn40nu35dmc.png', '-dpng', '-r400');
savefig(figure(1), 'dmc-ODP2ytestn40nu35.fig')
figure(2);
stairs(u);
print('utrajtestn40nu351dmc.png', '-dpng', '-r400');
savefig(figure(2), 'dmc-ODP2utestn40nu35.fig')