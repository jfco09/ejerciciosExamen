#include <stdio.h>
#include "imagen.h"
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

int main()
{
    //Recorre toda la imagen y lleva puntos negros a 0 y blancos a 255
    for(int f = 0; f < 480; f++ ){
        for(int c = 0; c < 640; c++){
            if(m[f * 640 + c] < 100) {
                m[f * 640 + c] = 0;
            }else{
                m[f * 640 + c] = 255;
            }
        }
    }

    //Variable que lleva la cuenta del numero de objetos
    int nObjetos = 0;
    //Bucle para recorrer todas las filas desde la 2º hasta el final
    for(int f = 1; f < 480 ; f++ ){
        //Variable si se ha detectado un objeto en la fila
        int detectado = 0;
        //Bucle que recorre todas las columnas
        for(int c = 0; c < 640 ; c++){
            //Detecta primer valor
            if(m[f * 640 + c] == 0 ) {
                //Guarda el valor de la columna detectada
                int ci = c;
                //Busca el borde siguiente
                while(m[f * 640 + c] == 0){
                    c++;
                }
                //Guarda la columna final del objeto
                int cf = c ;
                int caux = ci;
                //Busca en la fila anterior si se detecto un objeto entre los límites
                while(!detectado && caux < cf){
                    if(m[(f - 1) * 640 + caux] != 0 && m[(f - 1) * 640 + caux] != 255){
                        //Guarda el numero del objeto detectado
                        detectado = m[(f - 1) * 640 + caux];
                    }
                    caux++;
                }
                //Saber si se ha detectado algún objeto
                if (detectado == 0) {
                    //Si no se aumenta el contador
                    nObjetos ++;
                    //Se cambia los pixeles al valor del numero del objeto
                    for(int j = ci; j <= cf; j++)
                        m[f*640+j] = nObjetos;
                } else {
                    for(int j = ci; j <= cf; j++)
                        //Guarda los pixeles con el valor del objeto detectado
                        m[f*640+j] = detectado;
                }

            }
        }
    }
    //Crear objeto Objeto
    typedef struct {
        float xc, yc,L;
        int area;
    } Objeto;

    //Crrar array con 10 elementos del tipo Objeto
    Objeto elipses[10];
    //Recorre los objetos detecctados
    for(int obj = 0; obj < nObjetos ; obj++){
        //incializa a 0 las variables
        elipses[obj].xc = 0;
        elipses[obj].yc = 0 ;
        elipses[obj].area = 0;
        elipses[obj].L = 0;
        int puntos = 0;
        //Recorre la imagen en busca de los objetos
        for(int f = 0; f < 480 ; f++ ){
            for(int c = 0; c < 640 ; c++){
                //El n+1
                if(m[f * 640 + c] == obj + 1 ) {
                    //xc suma las columnas
                    elipses[obj].xc += c;
                    //yc suma las filas
                    elipses[obj].yc += f;
                    puntos++;

                }
            }
        }
        //xc e yc se calcula haciendo la media
        elipses[obj].xc /= puntos;
        elipses[obj].yc /= puntos;
        //El area es la suma de los puntos detectados
        elipses[obj].area += puntos;

        //Bucle que recorre la imagen
        for(int f = 0; f < 480 ; f++ ){
            for(int c = 0; c < 640 ; c++){
                if(m[f * 640 + c] == obj + 1) {
                    //Calcula la distancia del punto al centro
                    float dx = c - elipses[obj].xc;
                    float dy = f - elipses[obj].yc;
                    float d = sqrt(dx*dx + dy*dy);
                    //La distancia que se guarda es la mayor detectada
                    if (d > elipses[obj].L)
                        elipses[obj].L = d;

                }
            }
        }
        //Imprime los resultadosn en la consola
        printf("Objeto : %d  ", obj + 1);
        printf("xc: %.2f  ", elipses[obj].xc);
        printf("yc: %.2f  ", elipses[obj].yc);
        printf("L: %.2f  ", elipses[obj].L);
        printf("Area: %d\n", elipses[obj].area);
    }

    return 0;
}
