#include <iostream>
using namespace std;

//Declaración de variables globales
int amarillo  = 0xFFFF00;
int azul = 0x7FFFFF;
//Declaración de las funciones a usar
double entradaAnalogica(){};
void salidaAnalogica(double valor){};
int interruptor(){};

//Creacion de clase Temporizador
class Temporizador {
private:
    //Parametro t lleva el periodo )
    int t;
public:
    //Constructor de la clase
    Temporizador(int periodo) {
        //Actualiza el valor de t
        t = periodo;
    }

    //Funcion que realiza la espera
    void espera(){
        //Codigo que realiza la espera
    }
};

//Clase LCD
class LCD{
public:
    //Constructor de la clase
    LCD(){};
    //Metodo para dibujar el punto
    void dibujaPunto(int x, int y, int color){};
    //Metodo para borrar la pantalla
    void borra(){};
};


int main()
{
    double T = 0.1,Kp = 0.1, Ti = 10;
    Temporizador t(1/T);
    LCD lcd;

    double e = 0.0;
    double e1 = 0.0;

    double u = 0.0;
    double u1 = 0.0;
    double consigna = 0;
    double y = 0;
    int nPeriodos = 0;
    int nPuntos = 0;

    int consignas[480] = {};
    double salidas[480] = {};

    while(1){
        consigna = interruptor();
        y = entradaAnalogica();
        e = consigna - y;
        u = u1 + Kp * (1 + T/Ti) * e - Kp * e1;
        salidaAnalogica(u);

        u1 = u;
        e1 = e;

        for(int i = 0; i < 480 - 1; i++){
            consignas[i] = consignas[i + 1];
            salidas[i] = salidas[i + 1];
        }
        consignas[319] = consigna;
        salidas[319] = y;
        if(nPeriodos == 10){
            nPeriodos = 0;
            if(nPuntos < 480){
                lcd.dibujaPunto(nPuntos, (y + 0.5) * 319 / 2, amarillo);
                lcd.dibujaPunto(nPuntos, (consigna + 0.5) * 319 / 2, azul);
            }
            else{
                lcd.borra();
                for(int i = 0; i < 480; i++){
                    lcd.dibujaPunto(i, (salidas[i] + 0.5) * 319 / 2, amarillo);
                    lcd.dibujaPunto(i, (consignas[i] + 0.5) * 319 / 2, azul);
                }
            }
            nPuntos++;
        }

        nPeriodos ++;
        t.espera();
    }

    return 0;
}
