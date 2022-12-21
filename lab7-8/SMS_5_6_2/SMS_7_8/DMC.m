clear all
%% S
load('S');
S = s;
D = length(S)

%% Parametry do dostosowania

% Początkowa wartość lambda w testach: 0.05
% Wyznaczone optymalne parametry: N = 5; Nu = 1; lamb = 0.1;
N = 30;
Nu = 30;
lamb = 0.5;

%% Offline
M=zeros(N,Nu);
for i=1:N
 for j=1:Nu
  if (j<=i)             
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
%% 
Ke = sum(K(1,:))
Ku = K(1,:)*MP;

c_Ku = sprintf('%.6f,' , Ku);
c_Ku = c_Ku(1:end-1);% strip final comma
c_Ku = strcat('const float K_u[297] = {', c_Ku, '};')
