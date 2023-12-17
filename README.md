# ejerciciosExamen
 Ejercicios Preparacion Examen SACB

 ## Ejercicio1
 ej1) En una matriz m se dispone de 10 números enteros:
int m[] = { 5, 3, 2, 6, 3, 4, 1, 6, 4, 8 };
Codifica un algoritmo en C que calcule cuál es el valor mínimo. 
 ```c
#include <stdio.h>
// Declaracion del vector m global
int m[] = { 5, 3, 2, 6, 3, 4, 1, 6, 4, 8 };


int main()
{
    // Una forma de calcular el tamaño del vector de forma automatica
    int N = sizeof(m) / sizeof(m[0]);
    //Supone que le minimo es el primer valor
    int minimo = m[0];
    //Bucle que recorre toda el vector
    for(int i = 0; i < N; i++){
        //Si hay algun valor menor que el candidato -> minimo
        if(m[i] < minimo){
            minimo = m[i];
        }
    }
    //Imprime el resultado
    printf("Minimo es %d\n", minimo);
    return 0;
}

```
## Ejercicio 2
ej2) En una matriz se dispone de varios valores enteros:
int m[] = { 15, 13, 2, -6, 17, 4, 1, 6, -9, 8 }; // Matriz con los valores
int nElementos = 10; // Número de elementos en la matriz
Codifica un algoritmo en lenguaje C que determine en variables
int posMinimo, posMaximo; // Posición de mínimo y máximo en la matriz
en qué posición está el valor mínimo y en qué posición está el valor máximo. En este ejemplo el mínimo
está en la posición 8 y el máximo en la 4. 
```c
#include <stdio.h>
int m[] = { 15, 13, 2, -6, 17, 4, 1, 6, -9, 8 }; // Matriz con los valores
int nElementos = 10; // Número de elementos en la matriz
int posMinimo, posMaximo = 0; // Posición de mínimo y máximo en la matriz
int main()
{
    //Recorre toda la matriz
    for(int i = 0; i < nElementos; i++){
        //Si un valor es menor que el candidato guarda la posicion
        if(m[i] < m[posMinimo]){
            posMinimo = i;
        }
        //Si un valor es mayor que el candidato guarda la posicion
        if (m[i] > m[posMaximo]){
            posMaximo = i;
        }
    }
    //Muestra en consola los valores
    printf("PosMinimo %d  ",posMinimo );
    printf("Minimo %d\n", m[posMinimo]);
    printf("PosMaximo %d  ",posMaximo);
    printf( "Maximo %d\n", m[posMaximo]);
    return 0;
}


```
## Ejercicio 3
ej3) En una matriz se dispone de varios valores enteros:
int m[] = { 15, 13, 2, -6, 17, 4, 1, 6, -9, 8 }; // Matriz con los valores
int nElementos = 10; // Número de elementos en la matriz

Codifica un algoritmo en lenguaje C que los ordene de menor a mayor.
Para ello se puede determinar la posición donde se encuentra el valor mínimo de todos los valores, desde la posición 0 al final. Una vez encontrada la posición del valor mínimo, se intercambia el valor mínimo con el de la posición 0. Luego se hace lo mismo, pero analizando desde la posición 1 al final. Y  así sucesivamente.
```c
#include <stdio.h>
//Declaracion de variables

int m[] = { 15, 13, 2, -6, 17, 4, 1, 6, -9, 8 }; // Matriz con los valores
int nElementos = 10; // Número de elementos en la matriz
int aux;

int main()
{
    //Bucle for que recorre de 0 hasta 7 (la ultima pareja de datos no hace falta intercambiarla)
    for (int j = 0; j <  nElementos - 1; j++){
        //supone minimo como laposicion j
        int posMinimo = j;
        //busca el minimo desde la posicion j + 1 hasta el final, j+1 es para evitar compararlo consigo mismo (opcinal)
        for(int i = j+1; i < nElementos; i++){
            if(m[i] < m[posMinimo]){
                posMinimo = i;
            }
        }
        //Intercambia los valores, necesario una variable aux
        aux = m[j];
        m[j] = m[posMinimo];
        m[posMinimo] = aux;
    }

    //Imprime la matriz resultante
    printf("M = ");
    for (int j = 0; j <  nElementos; j++){
        printf("%d ", m[j] );

    }
    printf("\n");
    return 0;
}

```
## Ejercicio 4
ej4) En una matriz m1 se dispone de 10 números enteros:
int m[] = { 5, 3, 2, 6, 3, 4, 1, 6, 4, 8 };
Codifica en C un algoritmo que ordene los valores de menor a mayor utilizando el método de la burbuja.
Se hacen pasadas sucesivas por la matriz y se intercambian los valores de dos posiciones consecutivas cuando están desordenados. El algoritmo finaliza cuando no se encuentran valores consecutivos desordenados.

```c
#include <stdio.h>


int main()
{
    int m[] = { 5, 3, 2, 6, 3, 4, 1, 6, 4, 8 };
    //Calcula el tamaño de la matriz
    int N = sizeof(m) / sizeof(m[0]);
    //Variable que me dice cuando los valores ya no estan desordenados
    int desordenados = 1;
    //Variable auxiliar para almacenar un valor durante el intercambio
    int aux;

    //Bucle while que se ejecuta hasta que todos los valores esten ordenados
    while(desordenados){
        //Variable para saber si se ha hecho un intercambio
        int desorden = 0;
        //Bucle for que va buscando e intercambiando valores
        for(int i = 0; i < N - 1; i++){
            if(m[i] < m[i +1]){
                desorden = 1;
                aux = m[i];
                m[i] = m[i + 1];
                m[i + 1] = aux;
            }
        }
        //Si no hay ningun valor desordenado -> fin del bucle
        if (desorden == 0){
            desordenados = 0;
        }
    }


    //Imprime la matriz resultante
    printf("M = ");
    for (int j = 0; j <  N; j++){
        printf("%d ", m[j] );

    }
    printf("\n");
    return 0;
}


```
## Ejercicio 4 del boletin de clase (VECTORES)
Codifica herramientas para el manejo de puntos y vectores en
Se dispondrá de las siguientes operaciones:
- módulo de un vector
- vector unitario en la dirección de un vector
- suma de dos vectores y
- resta de dos vectores y
- multiplicación de un vector por un escalar
- producto escalar de dos vectores y
- producto vectorial de dos vectores

Codifica también un programa para generar la secuencia de 20 posiciones donde tiene que situarse sucesivamente un robot que tiene que realizar la sutura de dos tejidos. El robot tiene que situarse a una distancia de 0.2 unidades de la línea de unión de ambos tejidos representados por dos planos y a la misma distancia de ambos. Hay que generar también en qué dirección hay que hacer apuntar el cabezal del robot. El primer plano contiene los puntos p1, p2 y p3 cuyas coordenadas se indican a continuación. El segundo plano contiene a p1, p2 y p4. La línea de unión de ambos planos va de p1 a p2, el robot tiene que situarse sobre p1 hasta situarse sobre p2.

Vectores: p1 = (5.07, 3.28, 2.16) p2 = (10.53, 12.19, 17.72) p3 = (21.01, 15.63, 6.97) p4 = (-3.15, 4.96, 0.32)

```c
#include <stdio.h>
#include <math.h>

//Creacion del tipo de dato Vector
typedef struct{
    float x,y,z;
}Vector;

float modulo(Vector v){
    float mod = sqrt(v.x * v.x + v.y * v.y + v.z * v.z );
    return mod;
}
Vector unitario(Vector v){
    float mod = modulo(v);
    Vector u;
    u.x = v.x / mod;
    u.y = v.y / mod;
    u.z = v.z / mod;
    return u;
}

Vector suma(Vector v1, Vector v2){
    Vector v;
    v.x = v1.x + v2.x;
    v.y = v1.y + v2.y;
    v.z = v1.z + v2.z;
    return v;
}

Vector resta(Vector v1, Vector v2){
    Vector v;
    v.x = v1.x + v2.x;
    v.y = v1.y + v2.y;
    v.z = v1.z + v2.z;
    return v;
}

Vector mescalar(Vector v, float a){
    Vector vescalar;
    vescalar.x = v.x * a;
    vescalar.y = v.y * a;
    vescalar.z = v.z * a;
    return vescalar;
}

float pescalar(Vector v1, Vector v2){
    float producto = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    return producto;
}
Vector pvectorial(Vector v1, Vector v2){
    Vector v;
    v.x = v1.y * v2.z - v2.y * v1.z;
    v.y = v2.x * v1.z - v1.x * v2.z;
    v.z = v1.x * v2.y - v2.x * v1.y;
    return v;
}

void calcularPuntos(Vector p1,Vector p2,Vector p3,Vector p4, Vector *puntos, Vector * direccion, float separacion, int nPuntos){
    Vector v13 = resta(p3, p1);
    Vector v12 = resta(p2, p1);
    Vector v14 = resta(p4, p1);

    Vector v123 = pvectorial(v13, v12);
    Vector v124 = pvectorial(v12, v14);

    Vector u123 = unitario(v123);
    Vector u124 = unitario(v124);

    Vector b = suma(u123, u124);
    Vector bu = unitario(b);

    Vector desplazamiento = mescalar(bu, separacion);

    float d12 = modulo(v12);
    float d = d12 / 19;
    Vector u12 = unitario(v12);
    Vector vd = mescalar(u12, d);

    puntos[0] = suma(p1, desplazamiento);
    for(int i = 1; i < nPuntos; i++)
        puntos[i] = suma(puntos[i-1], vd);

    * direccion = mescalar(bu, -1);

}

int main()
{
    //Inicilización vectores
    Vector p1,p2,p3,p4;
    p1.x = 5.07;
    p1.y = 3.28;
    p1.z = 2.16;

    p2.x = 10.53;
    p2.y = 12.19;
    p2.z = 17.72;

    p3.x = 21.01;
    p3.y = 15.63;
    p3.z = 6.97;

    p4.x = -3.15;
    p4.y = 4.96;
    p4.z = 0.32;

    //Crear 20 puntos de tipo Vector y la direccion del robot
    Vector puntos[20], direccion;
    calcularPuntos(p1, p2, p3, p4, puntos, & direccion,0.2,20);

    // Imprimir la secuencia de puntos y la dirección
    printf("Secuencia de Puntos:\n");
    for (int i = 0; i < 20; ++i) {

        printf("(%.2f, %.2f, %.2f)\n", puntos[i].x, puntos[i].y, puntos[i].z);
    }

    printf("\nDireccion del Robot:\n");
        printf("(%.2f, %.2f, %.2f)\n", direccion.x, direccion.y, direccion.z);


    return 0;
}

```

## Ejercicio 14
Codifica una subrutina para un controlador digital PID industrial gobernado por un microcontrolador con el objetivo de realizar el ajuste automático (autotuning) de sus parámetros
```c
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

```
## Ejercicio 15 (ej6 en el boletin)

Codifica una aplicación en C++ para relizar un control PI. Se controla una planta SISO, utilizando:
- una entrada analógica para medir la salida de la planta mediante double entradaAnalogica().
- una salida analógica para fijar la actuación con la función void salidaAnalogica(double valor).
- un interruptor que se puede consultar con la función int interruptor(), que devuelve un buleano cierto si está en posición ON y falso si está en posición OFF. Un operario acciona este interruptor para fijar una consigna para la planta de valor 1 en posición ON y de valor 0 en posición OFF. 
```cpp
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


```

## Ejercicio 29
Codifica en lenguaje C un módulo bloquememoria.c con su correspondiente archivo de declaraciones bloquememoria.h donde se definan herramientas para la reserva dinámica de bloques de memoria en un microcontrolador con memoria RAM de tamaño reducido, de manera que se puede utilizar como se indica en este ejemplo:
```c
#include "bloquememoria.h"
uint8_t m[65536];
// Variable donde se van a realizar las operaciones
int main() {
 TBloqueMemoria zona; // Representa a una zona de memoria donde se van a realizar reservas
 inicializaBloqueMemoria(m, 65536, &zona);
 // Crea un objeto para manejar una zona de memoria en la variable 'm'
 // con un tamaño de 65536 bytes
 double* pd = (double*) reservaBloqueMemoria(100 * sizeof(double), & zona);
 // Reserva un bloque de memoria para almacenar 100 reales de 64 bits. Devuelve
 // la dirección donde se almacena el primer número real.
 pd[0] = 3.25; // Guarda un dato en el bloque

 uint16_t* p16 = (uint16_t*) reservaBloqueMemoria(5 * sizeof(uint16_t), & zona);
 // Reserva un bloque de memoria para almacenar 5 enteros de 16 bits sin signo.
 // Devuelve la dirección del primer entero.

 p16[0] = 14; // Guarda un dato en el bloque

 if (errorReservandoBloqueMemoria(& zona)) // Si hubo algún error durante las reservas,
 while(1); // bloquea la ejecución del programa
 liberaBloquesMemoria(& zona);
 // Libera todos los bloques de memoria reservados
 uint32_t* p32 = (uint32_t*) reservaBloqueMemoria(30 * sizeof(uint32_t), & zona);
 // Reserva un bloque de memoria para almacenar 30 enteros de 32 bits sin signo.
 // Devuelve la dirección del primero de ellos.
 p32[0] = 25; // Guarda un dato en el bloque

 return 0;
}
```

bloquememoria.h:
```c
#ifndef BLOQUEMEMORIA_H
#define BLOQUEMEMORIA_H
#include <stdint.h>

//Declaracion de la estructura
typedef struct{
    uint8_t * inicio;
    uint32_t tamaño;
    uint32_t reservados;
    int error;

}TBloqueMemoria;
//Declaracion de las funciones que se proporcionan en el main
void inicializaBloqueMemoria(uint8_t *m, uint32_t tamaño, TBloqueMemoria *bm);

void* reservaBloqueMemoria(uint32_t tamaño, TBloqueMemoria *bm);

void liberaBloquesMemoria(TBloqueMemoria *bm);

int errorReservandoBloqueMemoria(TBloqueMemoria *bm);

#endif // BLOQUEMEMORIA_H

```
bloquememoria.c:
```c

#include "bloquememoria.h"

//Funcionamiento de incializarbloque
void inicializaBloqueMemoria(uint8_t *m, uint32_t tamaño, TBloqueMemoria *bm){
    //Guarda en el bloque la posicion inicial
    bm->inicio = m;
    //Guarda en el bloque el tamaño total
    bm->tamaño = tamaño;
    //Inicializa a 0 el bloque reservado
    bm->reservados = 0;
}

void* reservaBloqueMemoria(uint32_t tamaño, TBloqueMemoria *bm){
    //comprueba si hay tamaño disponible si no lo hay devuelve error
    if(tamaño > bm->tamaño){
        bm->error = 1;

    }
    //Si no hay error reserva el tamaño
    if(bm->error == 0){
        bm->reservados = bm->reservados + tamaño;

    }
    //Devuelve direccion de inicio del bloque reservaod (inicio del bloque original + los reservados - el tamaño del bloque que se ha reservado ahora)
    return bm->inicio + bm->reservados - tamaño;
}

//Borra todos los reservados
void liberaBloquesMemoria(TBloqueMemoria *bm){
    bm->reservados = 0;
    bm->error = 0;
}
//Devuelve si hay un error en el bloque de memoria
int errorReservandoBloqueMemoria(TBloqueMemoria *bm){
    return bm->error;
}

```
main.c:
```c
#include <stdio.h>

#include "bloquememoria.h"

uint8_t m[65536];
// Variable donde se van a realizar las operaciones
int main() {
    TBloqueMemoria zona; // Representa a una zona de memoria donde se van a realizar reservas
    inicializaBloqueMemoria(m, 65536, &zona);
    // Crea un objeto para manejar una zona de memoria en la variable 'm'
    // con un tamaño de 65536 bytes
    double* pd = (double*) reservaBloqueMemoria(100 * sizeof(double), & zona);
    // Reserva un bloque de memoria para almacenar 100 reales de 64 bits. Devuelve
    // la dirección donde se almacena el primer número real.
    pd[0] = 3.25; // Guarda un dato en el bloque

    uint16_t* p16 = (uint16_t*) reservaBloqueMemoria(5 * sizeof(uint16_t), & zona);
    // Reserva un bloque de memoria para almacenar 5 enteros de 16 bits sin signo.
    // Devuelve la dirección del primer entero.

    p16[0] = 14; // Guarda un dato en el bloque

    if (errorReservandoBloqueMemoria(& zona)) // Si hubo algún error durante las reservas,
        while(1); // bloquea la ejecución del programa
    liberaBloquesMemoria(& zona);
    // Libera todos los bloques de memoria reservados
    uint32_t* p32 = (uint32_t*) reservaBloqueMemoria(30 * sizeof(uint32_t), & zona);
    // Reserva un bloque de memoria para almacenar 30 enteros de 32 bits sin signo.
    // Devuelve la dirección del primero de ellos.
    p32[0] = 25; // Guarda un dato en el bloque

    return 0;
}

```

## Ejercicio 32
 En una factoría de fabricación de productos farmacéuticos hay 10 máquinas idénticas, desde la máquina 1 hasta la máquina 10. Para ejecutar la próxima orden de producción en una máquina determinada, se elige la máquina libre que menos órdenes ha ejecutado previamente. 
 Codifica un algoritmo en lenguaje C que determine la máquina que tiene que ejecutar la siguiente orden, teniendo en cuenta la información indicada en las siguientes variables: 
```c
#include <stdio.h>
int ordenes[] = { 1, 3, 2, 6, 3, 4, 1, 6, 4, 8 }; // Número de órdenes ejecutadas por cada máquina
int libres[] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // Buleanos que indican qué máquinas están libres
int nMaquinas = 10; // Número de máquinas existentes

int main()
{
    int maquina = 0;
    int primera = 1;

    for(int i = 0; i < nMaquinas ; i++){
        //Encuentra la primera maquina libre
        if(libres[i] == 1 && primera){
            maquina = i;
            primera = 0;
        }
        //Compara maquinas libres para encontrar la que menor ordenes tenga
        if(libres[i] == 1 && primera != 0){
            if(ordenes[i] < ordenes[maquina]){
                maquina = i;
            }
        }

    }

    //Imprimir resultados
    if(primera == 0){
        printf("Maquina a usar: %d ", maquina);
        printf("Ordenes: %d\n", ordenes[maquina]);
    }else{
        printf("Todas las maquinas estan ocupadas\n");
    }

    return 0;
}

```

## Ejercicio 33
Un sistema embebido está conectado a una cámara de vídeo de 640x480 puntos de resolución.
Cada punto de una imagen se representa en niveles de gris mediante un byte sin signo (desde 0=negro hasta 255=blanco). Se supone que la imagen ya se ha capturado y todos los bytes están disponibles en una matriz uint8_t m[307200] creada en el archivo imagen.h 
 ministrado.
Todos los bytes se organizan en esta matriz por filas de puntos en la imagen, desde la fila superior a la inferior y dentro de cada fila, de izquierda a derecha. Codifica un algoritmo que calcule una serie de características de objetos elipsoidales (de color oscuro, con valor menor que 100) existente en la escena (el fondo es de color claro, mayor que 200). Para cada objeto hay que obtener su posición xc, yc, su dimensión máxima L. y su área (número de puntos ocupados por el objeto).

```c
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

```
