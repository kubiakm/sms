#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define D 30
#define N 20
#define Nu 10
#define lambda 10

#define sim_time 180



int main() {
    // Wczytywanie modelu odpowiedzi skokowej
    float s[60] = {0.0, 0.0, 0.0, 0.0252, 0.06790896, 0.122148105408, 0.1833221578454784, 0.2479477572227568, 0.3134299052923838, 0.37787861828914715, 0.43995915046593026, 0.49877016304147265, 0.553745083987218, 0.6045726509516768, 0.6511332683621048, 0.693448354534888, 0.7316403180450126, 0.7659011957575681, 0.7964683175775173, 0.8236056436849274, 0.8475896562614025, 0.8686988859786772, 0.8872063194484523, 0.9033740722965894, 0.917449827725374, 0.9296646359754195, 0.9402317490888716, 0.9493462304535176, 0.9571851320286341, 0.9639080758355963, 0.9696581118655522, 0.9745627533881085, 0.9787351138989687, 0.982275088596534, 0.9852705381461484, 0.9877984442605878, 0.9899260158676174, 0.9917117318248148, 0.9932063116720115, 0.9944536101078584, 0.9954914340069502, 0.9963522830773212, 0.9970640168744397, 0.9976504519831292, 0.9981318938714108, 0.9985256083054139, 0.9988462373685998, 0.9991061651124049, 0.999315837727112, 0.9994840428989779, 0.999618152741667, 0.9997243343793132, 0.9998077319320597, 0.9998726233253493, 0.999922555020719, 0.9999604574548631, 0.9999887436795842, 1.000009393420658, 1.0000240245200305, 1.000033953493674};
    int i, j, k;

    // Inicjalizacja zmienych wykorzystywanych w obliczeniach online
    float dUP[D-1] = {0};
    float Y_zad[N] = {0};
    float Y[N] = {0};
    float Y0[N] = {0};
    float dU[Nu] = {0};
    //Pętla symulacji
    // Parametry obiektu (Jakiegoś losowego)
    float i1 = 0.09;
    float i2 = 0.28;
    float prev_dy, dy, ddy, real_dy;
    float y[sim_time] = {0};
    float u[sim_time] = {0};
    float y_zad = 1.0;
    for (int t=3; t < sim_time; ++t) {
        if (t > sim_time / 2) y_zad = 2.0;
        // Symulacja obiektu (Jakiś wymyślony)
        dy = (u[t-3] - y[t-1]) * i1;
        ddy = (dy - prev_dy) * i2;
        real_dy = prev_dy + ddy;
        y[t] = y[t-1] + real_dy;
        prev_dy = real_dy;

        for (i=D-2; i>0; --i)
            dUP[i] = dUP[i-1];

        dUP[0] = u[t-1] - u[t-2];

        for (i=0; i < N; ++i)
            Y_zad[i] = y_zad;

        for (i=0; i < N; ++i)
            Y[i] = y[t];

        for(i=0; i < N; ++i) Y0[i] = 0;

        for(i=0; i < N; ++i)
			for(k=0; k < D-1; ++k) 
				Y0[i]+=MP[i][k]*dUP[k];

        for (i=0; i < N; ++i)
            Y0[i] += Y[i];
        for (i=0; i < N; ++i)
            Y_zad[i] -= Y0[i];

        for(i=0; i < Nu; ++i) dU[i] = 0;

        for(i=0; i < Nu; ++i)
			for(k=0; k < N; ++k) 
				dU[i]+=K[i][k]*Y_zad[k];

        u[t] = u[t-1] + dU[0];
    }
    // Zapisanie przebiegów testowych
    FILE *fptr;
    fptr = fopen("y.out", "w");
    if (fptr == NULL) {
        printf("Error!");
        exit(1);
    }
    for (int k=3; k < sim_time; ++k)
        fprintf(fptr, "%.4f\n", y[k]);
    fclose(fptr);
    return 0;
}