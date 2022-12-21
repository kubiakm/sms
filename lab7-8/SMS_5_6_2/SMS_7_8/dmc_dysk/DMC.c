const short int Nu= 4; //horyzont sterowania
const short int N = 4; //horyzont predykcji
const short int lambda = 0;
const short int D =2; //horyzont dynamiki

double deltaU =0;
double deltaUP[D-1];
double s[30];
double Ku[D-1]; //tu wyniki z matlaba
double Ke;
float DMC(float u,float y)
{	
	double e = 0, Ku_deltaUP =0;
	e = y_zad - y;
	for(int i = 0; i< D-1 ;i++)
		Ku_deltaUP += Ku[i]*deltaUP[i];
	deltaU = Ke*e - Ku_deltaUP;
	
	for(int i=D; i>0 ;i--)
		deltaUP[i]= deltaUP[i-1];
	deltaUP[0] = deltaU;
	
	return u_prev1 +deltaU;
}
