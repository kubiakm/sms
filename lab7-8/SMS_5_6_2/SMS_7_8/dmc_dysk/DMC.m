lambda=1; %parametr lambda np. 1
D=12; %horyzont dynamiki (D)
N=12;%horyzont predykcji (N)
Nu=6; %horyzont sterowania (Nu)(ilosc przyszlych przyrostow wartosci sterowania)

s=[1 2 3 4 5 6 7 8 9 10 11 12];
%macierz wspó³czynników odpowiedzi skokowej wymiary(NxNu)
M=zeros(N,Nu); 
for i=1:N
 for j=1:Nu
  if (j<=i)             %wypelnianie macierzy trojkatnej dolnej M   
   M(i,j)=s(i-j+1);
  end;
 end;
end;

I=eye(Nu);              %tworzenie macierzy jednostkowej o wymiarach NuxNu
K=(M'*M+lambda*I)\M';   %macierz K
Mp=zeros(N,D-1);        %macierz ma wymiary Nx(D-1)
%wypelnianie macierzy Mp
for i=1:N
 for j=1:D-1
  if i+j<=D
   Mp(i,j)=s(i+j)-s(j);
  else
   Mp(i,j)=s(D)-s(j);
  end;      
 end;
end;
Ke=sum(K(1,:));         %wspó³czynnik Ke
Ku=K(1,:)*Mp;           %wspó³czynnik Ku
