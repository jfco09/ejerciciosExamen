#include <stdio.h>
#include <math.h>
//Variables globales
double entradaAnalogica(int nEntrada){};
void salidaAnalogica(double valor){};
unsigned int MS = 0;
float pi = 3.141592;

//Subrutina para ajustar PID
void autotuningPID(double actuacion,double * Kp, double *Ki, double *Kd){
    //Primero poner salida a 5V
    salidaAnalogica(actuacion);
    //Esperar a que llege a 0.5
    while(entradaAnalogica(1) < 0.5);
    //Poner salida a -5V
    salidaAnalogica(-actuacion);
    //Varibale que inidica si la planta está estable
    int estable = 0;
    //Guardan los 5 valores del periodo y de Amplitud
    double P[5] =  {};
    double A[5] = {};

    //Variables que guarda las medias
    double mediaP = 0.0;
    double mediaA = 0.0;

    //Variable que guarda la salida del sistema
    double y = 0.0;

    int periodos = 0;
    while(!estable){
        //Reinicializa el contador a 0
        MS = 0;
        //Establece el maximo a 0
        double max = 0;
        //Bucle que espera a que la señal sea positiva
        do {
            //Guarda la salida en y
            y = entradaAnalogica(1);
            //Busca el maximo
            if (y > max){
                max = y;
            }

        } while (y > 0);
        //Establece la salida a +5V
        salidaAnalogica(actuacion);

        double min = 0;
        //Bucle que espera a que la señal sea negativa
        do{
            //Guarda la salida en y
            y = entradaAnalogica(1);
            //Busca el minimo
            if (y < min){
                min = y;
            }
        }while(y < 0);
        //Establece la salida a -5V
        salidaAnalogica(-actuacion);
        //Fin del ciclo
        periodos++;

        //Desplaza los valores en los arrays
        for(int i = 0; i < 4;i++){
            P[i] = P[i+1];
            A[i] = A[i+1];
        }
        //Guarda el valor actual en la ultima posicion
        P[4] = MS / 1000.0;
        A[4] = max - min;

        //Cada 5 periodos calcular si ya está estable
        if (periodos == 5){
            //Incializar la suma
            double sumaP = 0;
            double sumaA = 0;

            //Sumar los valores previos
            for (int i = 0; i < 5; i++){
                sumaP += P[i];
                sumaA += A[i];
            }

            //Hacer la media
            mediaP = sumaP / periodos;
            mediaA = sumaA / periodos;


            estable = 1;
            for (int i = 0; i < 5; i++){
                //Si algún valor difiere, la planta no está estable
                if(fabs(P[i] - mediaP) > 0.2 || fabs(A[i] - mediaA) > 0.2 ){
                    estable = 0;
                }
            }
            //Restablecer contador periodos
            periodos = 0;
        }
    }
    //Una vez la planta está estable, calcula parametros PID
    double Ku = (4 * 5) / pi * mediaA;
    *Kp = 0.6 * Ku;
    *Ki = 1.2 * Ku / mediaP;
    *Kd = 0.075 * Ku;
}

int main()
{
    double Kp, Ki,Kd;
    double D = 5;
    //Llamada a la subrutina
    autotuningPID(D,&Kp,&Ki,&Kd);
    return 0;
}
