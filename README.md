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

!! A pesar de usar la solución del profe, no me da los valores que tienen que dar
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
                if(fabs(P[i] - mediaP) > 0.02 * mediaP|| fabs(A[i] - mediaA) > 0.02 * mediaA ){
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
public:
    //Constructor de la clase
    Temporizador(int t) {};

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
    Temporizador t(100);
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
        consignas[479] = consigna;
        salidas[479] = y;
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
        if(libres[i] == 1 && primera == 0){
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
## Ejercicio 34
Implementar el filtro butter usando clases declaradas en un archivo filtrobutter.h y programando la funcionalidad en filtrobutter.cpp

filtrobutter.h

```cpp

#ifndef FILTROBUTTER_H
#define FILTROBUTTER_H


class FiltroButter
{
private:
    float y[6];
    float u[6];
public:
    FiltroButter();
    float filtrar(float *señal);
};

#endif // FILTROBUTTER_H


```
filtrobutter.cpp:

```cpp
#include "filtrobutter.h"

FiltroButter::FiltroButter(){};

float FiltroButter::filtrar(float *señal){
    for(int i = 0; i < 5; i++){
        y[i] = y[i + 1];
        u[i] = u[i + 1];
    }
    u[5] = * señal;

    y[5] = 2.9754 * y[4] - 3.8060 * y[3] + 2.5453 * y[2] - 0.8811 * y[1] + 0.1254*y[0]  + 0.0013 * u[5] + 0.0064 * u[4] + 0.0128 * u[3] + 0.0128 * u[2] + 0.0064 * u[1] + 0.0013 * u[0];

    return y[5];
}

```


main.cpp:
```cpp
#include <iostream>
#include "filtrobutter.h"
using namespace std;

void arrancaTemporizador(int s, int ns);
float mideSenal();
void esperaTemporizador();
void borraPantalla();
void dibujaLinea(int x1, int y1, int x2, int y2);

int main()
{
    float ys[4000] = {};
    float señal ;
    FiltroButter filtro;
    arrancaTemporizador(0,1000000);
    int nPeriodos = 0;
    int nPuntos = 0;
    while(1){
        señal = mideSenal();
        for(int i  = 0; i < 3999; i++  ){
            ys[i] = ys[i +1];
        }
        ys[3999] = filtro.filtrar( & señal);

        if(nPeriodos == 2000 && nPuntos < 4000){
            nPeriodos = 0;
            borraPantalla();
            for(int i  = 0; i < 3998; i++  ){

                dibujaLinea(i * 1023/3999, (ys[i] + 3) *(717 - 50)/6 + 50,(i +1) * 1023/3999 ,(ys[i + 1] + 3) *(717 - 50)/6 + 50) ;
            }


        }

        nPuntos++;
        esperaTemporizador();
        nPeriodos++;
    }


    return 0;
}


```

## Ejercicio 5
Autoajustado de un PID
```c
#include <stdio.h>
#include <math.h>
unsigned int MS = 0;
double entradaAnalogica(int nEntrada);
void salidaAnalogica(double valor);

void autotuning(double D, float * Kp, float * Ki, float *Kd){
    int ajustado = 0;

    salidaAnalogica(D);
    while(entradaAnalogica(1) < 0.5);
    salidaAnalogica(-D);
    float P[5] = {};
    float A[5] = {};
    float mediaP, mediaA;
    int nValores = 0;

    while(!ajustado){
        MS = 0;
        nValores ++;
        float maximo = 0;
        while(entradaAnalogica(1) > 0){
            if(entradaAnalogica(1) > maximo){
                maximo = entradaAnalogica(1);

            }
        }
        salidaAnalogica(D);

        float minimo = 0;
        while(entradaAnalogica(1) < 0){
            if(entradaAnalogica(1) < minimo){
                minimo = entradaAnalogica(1);
            }
        }
        salidaAnalogica(-D);

        for(int i = 0; i < 4; i++){
            P[i] = P[i + 1];
            A[i] = A[i + 1];
        }
        P[4] = MS;
        A[4] = maximo - minimo;
        if (nValores > 5){
            for( int i = 0; i < 5; i++ ){
                mediaP += P[i];
                mediaA += A[i];
            }
            mediaP /= 5.0;
            mediaA /= 5.0;

            ajustado = 1;
            for(int i = 0; i < 5; i++){
                if(fabs(P[i] - mediaP) > 0.02 || fabs(A[i] - mediaA) > 0.02 ){
                    ajustado = 0;
                }
            }

        }

    }
    float Ku = 4 * D / 3.14 * mediaA;
    *Kp = 0.6 * Ku;
    *Ki = 1.2 * Ku/ mediaP;
    *Kd = 0.075 * Ku;

}

int main()
{
    float Kp, Ki, Kd;
    autotuning(5, &Kp, &Ki, &Kd );

    return 0;
}


```
## Ejercicio 8 El filtro de Kuwahara

```c
#include <stdio.h>
#include <stdint.h>
#include <math.h>

//Variables globales

//Imagen sin filtrar (se supone que nos lo dan)
uint8_t imagen[];
//Variables que guardan el alto y ancho de la imagen (se supone que nos lo dan)
int ancho;
int alto;
//Grado del filtro (se supone que nos lo dan)
int L;

void calcularSector(int filai, int filaf,int columnai, int columnaf, double *desviacion, double * media){
    //Inicializa la suma y el nº de puntos
    double suma = 0 ;
    int nPuntos = 0;

    //Recorre al ventana sumando los valores
    for(int fi = filai; fi <= filaf; fi++ ){
        for(int ci = columnai; ci <= columnaf; ci++ ){
            suma += imagen[fi * alto + ci * ancho];
            nPuntos ++;
        }
    }
    //Calcula la media y lo guarda en el puntero media
    *media = suma / nPuntos;
    //Restablece la suma
    suma = 0.0;
    //Calcula el sumatorio para calcular luego la desviacion tipica
    for(int fi = filai; fi <= filaf; fi++ ){
        for(int ci = columnai; ci <= columnaf; ci++ ){
            suma += (imagen[fi * alto + ci * ancho] - *media) * (imagen[fi * alto + ci * ancho] - *media);
        }
    }
    //Calcula la desviacion tipica total del sector y lo almacena en el puntero
    *desviacion = sqrt( ( 1 / (nPuntos - 1) )  * suma);
}

int main()
{
    //Imagen donde guardo los valores filtrados, mismo tamaño que imagen original
    uint8_t imagenFiltrada[ancho * alto];

    //Bucle que recorre todos los pixeles de la imagen
    for(int f = 0;f < alto; f++){
        for(int c = 0; c < ancho; c++){
            //Para cada inicializa dos matrices para guardar los valores de desviacion y media de cada sector
            double desviaciones[4];
            double medias[4];

            //No estoy en ningun borde
            if( (-2 * L + f) >= 0 && (2 * L + f) <= alto &&  (-2 * L + c) >= 0 && (2 * L + f) <= ancho ){

                //LLamada a la funcion para calcular cada sector
                //Sector1: de (-2L+f, -2L+c) -> (f,c)
                calcularSector(-2 * L + f, f, -2 * L + c, c, &desviaciones[0], &medias[0]);
                //Sector2: de (-2L+f, c) -> (f,2L + c)
                calcularSector(-2* L + f, f, c, 2* L + c, &desviaciones[1], &medias[1]);
                //Sector3: de (f, c) -> (2L +f,2L + c)
                calcularSector(f, 2* L + f, c,  2* L + c, &desviaciones[2], &medias[2]);
                //Sector4: de (f, -2L +c) -> (2L +f, c)
                calcularSector(f, 2* L + f, -2* L + c,  c, &desviaciones[3], &medias[3]);

                //Busco la desviacion minima de los sectores y me quedo con el indice
                int imin= 0;
                for(int i = 0; i < 4; i++){
                    if(desviaciones[i] < desviaciones[imin] ){
                        imin = i;
                    }
                }
                //Guardo en la imagen filtrada la media del sector con la minima desviacion tipica (habria que pasarlo a un entero de 8 bits que no se como se hace)
                imagenFiltrada[f * alto + c * ancho] = medias[imin];

            }


            //Comrpuebo si estoy en la banda superior de la imagen (no puedo calcular ni el sector 1 ni el 2)
            if((-2 * L + f) < 0){
                if((-2 * L + c) < 0){

                    //Estoy en la esquina superior izq solo sector 3
                    //Sector3: de (f, c) -> (2L +f,2L + c)
                    calcularSector(f, 2* L + f, c,  2* L + c, &desviaciones[2], &medias[2]);
                    imagenFiltrada[f * alto + c * ancho] =  medias[2];

                }
                if((2 * L + c) > ancho){
                    //Estoy en la esquina superior dcha solo sector 4
                    //Sector4: de (f, -2L +c) -> (2L +f, c)
                    calcularSector(f, 2* L + f, -2* L + c,  c, &desviaciones[3], &medias[3]);
                    imagenFiltrada[f * alto + c * ancho] =  medias[3];

                }

                if((-2 * L + c) >= 0 &&(2 * L + c) <= ancho){
                    //No esta en esquinas
                    //Sector3: de (f, c) -> (2L +f,2L + c)
                    calcularSector(f, 2* L + f, c,  2* L + c, &desviaciones[2], &medias[2]);
                    //Sector4: de (f, -2L +c) -> (2L +f, c)
                    calcularSector(f, 2* L + f, -2* L + c,  c, &desviaciones[3], &medias[3]);

                    //Busco la desviacion minima de los sectores y me quedo con el indice
                    int imin= 2;
                    for(int i = 2; i < 4; i++){
                        if(desviaciones[i] < desviaciones[imin] ){
                            imin = i;
                        }
                    }
                    //Guardo en la imagen filtrada la media del sector con la minima desviacion tipica (habria que pasarlo a un entero de 8 bits que no se como se hace)
                    imagenFiltrada[f * alto + c * ancho] = medias[imin];

                }
            }
            //Compruebo que estoy en la banda inferior (no se puede calcular ni S3 ni S4)
            if((2 * L + f) > alto){
                if((-2 * L + c) < 0){
                    //Estoy en la esquina inferior izq solo sector 2
                    //Sector2: de (-2L+f, c) -> (f,2L + c)
                    calcularSector(-2* L + f, f, c, 2* L + c, &desviaciones[1], &medias[1]);
                    imagenFiltrada[f * alto + c * ancho] =  medias[1];
                }
                if((2 * L + c) > ancho){
                    //Estoy en la esquina inferior dcha solo sector 1
                    //Sector1: de (-2L+f, -2L+c) -> (f,c)
                    calcularSector(-2 * L + f, f, -2 * L + c, c, &desviaciones[0], &medias[0]);
                    imagenFiltrada[f * alto + c * ancho] =  medias[0];

                }
                if((-2 * L + c) >=  0 && (2 * L + c) <= ancho){
                    //No esta en esquinas
                    //Sector1: de (-2L+f, -2L+c) -> (f,c)
                    calcularSector(-2 * L + f, f, -2 * L + c, c, &desviaciones[0], &medias[0]);
                    //Sector2: de (-2L+f, c) -> (f,2L + c)
                    calcularSector(-2* L + f, f, c, 2* L + c, &desviaciones[1], &medias[1]);

                    //Busco la desviacion minima de los sectores y me quedo con el indice
                    int imin= 0;
                    for(int i = 0; i < 2; i++){
                        if(desviaciones[i] < desviaciones[imin] ){
                            imin = i;
                        }
                    }
                    //Guardo en la imagen filtrada la media del sector con la minima desviacion tipica (habria que pasarlo a un entero de 8 bits que no se como se hace)
                    imagenFiltrada[f * alto + c * ancho] = medias[imin];

                }

            }
            //Compruebo si estoy en la banda izq
            if((-2 * L + c) < 0){
                //Me olvido de las esquinas las calculo por las filas
                if((-2 * L + f) >= 0 && (2 * L + f) <= alto){
                    //Sector2: de (-2L+f, c) -> (f,2L + c)
                    calcularSector(-2* L + f, f, c, 2* L + c, &desviaciones[1], &medias[1]);
                    //Sector3: de (f, c) -> (2L +f,2L + c)
                    calcularSector(f, 2* L + f, c,  2* L + c, &desviaciones[2], &medias[2]);

                    //Busco la desviacion minima de los sectores y me quedo con el indice
                    int imin= 1;
                    for(int i = 1; i < 3; i++){
                        if(desviaciones[i] < desviaciones[imin] ){
                            imin = i;
                        }
                    }
                    imagenFiltrada[f * alto + c * ancho] = medias[imin];
                }
            }
            if((2 * L + c) <= ancho){
                //Me olvido de las esquinas las calculo por las filas
                if((-2 * L + f) >= 0 && (2 * L + f) <= alto){
                    //Sector2: de (-2L+f, c) -> (f,2L + c)
                    calcularSector(-2* L + f, f, c, 2* L + c, &desviaciones[1], &medias[1]);
                    //Sector3: de (f, c) -> (2L +f,2L + c)
                    calcularSector(f, 2* L + f, c,  2* L + c, &desviaciones[2], &medias[2]);

                    //Busco la desviacion minima de los sectores y me quedo con el indice
                    int imin= 1;
                    for(int i = 1; i < 3; i++){
                        if(desviaciones[i] < desviaciones[imin] ){
                            imin = i;
                        }
                    }
                    imagenFiltrada[f * alto + c * ancho] = medias[imin];
                }
            }
        }

    }

    return 0;
}


```
## Ejercicio17 filtro Savitzky-Golay

```c
#include <stdio.h>

int ventana;
double y[]; //Mi señal
double yk []; //Salida
int N;
int main()
{
    if (ventana == 5){
        //Para los primeros valores la señal filtrada es igual a la original
        yk [0] = y[0];
        yk [0] = y[0];
        for(int i = 2; i < N - 2; i++){
            yk [i] = 1/35 * ( - 3 * y[i -2] + 12 * y[i -1] +17 * y[i] +12 * y[i + 1] - 3 * y[i +2]);
        }
        //A partir de N-2 la señal filtrada es igual a la original
        yk [N-2] = y[N-2];
        yk [N-1] = y[N-1];
    }
    if (ventana == 7){
        for(int i = 0; i <3; i++ ){
            yk [i] = y[i];
        }
        for(int i = 3; i < N - 3; i++){
            yk [i] = 1/21 * (-2 * y[i-3] + 3 * y[i-2] + 6 * y[i-1] + 7 * y[i] -2 * y[i+3] + 3 * y[i+2] + 6 * y[i+1]);
        }
        for(int i = N - 3; i < N; i++ ){
            yk [i] = y[i];
        }


    }
    if (ventana == 9){
        for(int i = 0; i < 4; i++ ){
            yk [i] = y[i];
        }
        for(int i = 4; i < N - 4; i++){
            yk[i] = 1/231 * ( -21 * y[i - 4] + 14 * y[i - 3] + 39 * y[i - 2] + 54* y[i - 1] + 59 * y[i]  -21 * y[i + 4] + 14 * y[i + 3] + 39 * y[i + 2] + 54* y[i + 1]);
        }
        for(int i = N - 4; i < N; i++ ){
            yk [i] = y[i];
        }

    }


    return 0;
}

```
## Ejercicio 20
regulador.h:
```cpp
#ifndef REGULADOR_H
#define REGULADOR_H


class Regulador
{
private:
    int N,M;
    double *bi, *ai;
    double *u, *e;
public:
    Regulador(int gradoM, int gradoN, double * bcoef, double *acoef);
    double actuacion(double entrada);
    //El destructor lo creo pq aparece en la solución
    ~Regulador();
};

#endif // REGULADOR_H
```
regulador.cpp:

```cpp
#include "regulador.h"
//Constructor de la clase
Regulador::Regulador(int gradoM, int gradoN, double * bcoef, double *acoef)
{
    //Asignar variables externas a las internas
    N = gradoN;
    M = gradoM;
    bi = bcoef;
    ai = acoef;
    //new double permite crear una matriz del tamaño especificado. Según chatGPT, para asignar memoria dinámicamente,luego es necesario usar el destructor
    u = new double[N];
    e = new double[N + 1];

    //Poner a ceros las entradas
    for( int i = 0; i < N; i++){
        u[i] = 0.0;
        e[i] = 0.0;
    }
    //entradasAnteriores tiene un valor extra de u lo inicializo a 0 aquí
    e[N] = 0.0;
}
//Lo pongo pq sale en la solucion y chatGPT dice que al usar memoria dinámica es necesario liberarla usando el destructor, entiendo que esto no lo exigirá
Regulador::~Regulador(){
    delete[] u;
    delete[] e;
}
double Regulador::actuacion(double entrada){
    //Desplazo los valores de las entradas
    for (int i = N; i > 0; --i) {
        e[i] = e[i-1];
    }
    //Actualizo la entrada actual
    e[0] = entrada;
    //Desplazo los valores de actuacion
    for(int i = N-1; i > 0; i-- ){
        u[i] = u[i-1];
    }

    //Calculo actuacion actual restablezco a 0 para poder hacer la suma
    u[0] = 0.0;
    for(int i = 0; i < M+1;i++){
        //[N - M + i] Se calcula desde n-m hasta n *Ver imagen en la solución*
        u[0] += bi[i] * e[N - M + i];
    }
    for(int i = 0; i < N ; i++){
        u[0] -= ai[i] * u[i];
    }

    //Devuelvo el resultado
    return u[0];
}

```
main.cpp (lo da el)

```cpp
#include <iostream>
#include "regulador.h"
using namespace std;
void esperaPeriodo(){};
double mideSalida(){};
double referencia(){};
void aplicaActuacion(double actuacion){};

double b[] = {0.35, 0.889, 1.5}; // Coeficientes del polinomio del numerador
double a[] = {0.99, 0.863, 0.6}; // Coeficientes del polinomio del denominador
int m = 2; // Grado del polinomio del numerador
int n = 3; // Grador del polinomio del denominador

int main() {

    Regulador regulador(m, n, b, a); // Crea un regulador digital SISO
    while(1) {
        esperaPeriodo(); // Bloquea hasta siguiente período de muestreo
        double e = referencia() - mideSalida(); // Obtiene la señal de error
        double u = regulador.actuacion(e); // Calcula la actuación a partir del error
        aplicaActuacion(u); // Aplica la actuación a la planta
    }
    return 0;
}

```
## Ejercicio 25
main.cpp:
```cpp
#include <iostream>

using namespace std;
double entradaAnalogica(int nEntrada){};
void salidaAnalogica(int nSalida, double valor){};
void establecePrioridad(int prioridad);
class Temporizador{
public:
    int t;
    Temporizador(int periodo){
        t = periodo;
    };
    void espera();
};

class Canal
{
public:
    void *paquete;
    Canal() {}
    void envia(void * p, int n){
        paquete = p;
    };
    int recibe(void *p);
};

class Controlador
{
private:
    int nControlador;
    double Kp, Ti, T;
    int parametrosOk = 0;
    int consignaOk = 0;
    double R = 0.0;
    double a = 0.0;
    double uk = 0.0, uk1 = 0.0;
    double ek = 0.0;
    double entrada;
    double envio[2];
public:
    Controlador(int n) {
        nControlador = n;
    };
    void recibeParametros(double Kp,double Ti,double T){
        //Asigno variables externas a las internas
        this->Kp = Kp;
        this->Ti = Ti;
        this->T = T;
        parametrosOk = 1;
    };
    void recibeConsigna(double consigna){
        consignaOk = 1;
        R = consigna;
    };
    void actuacion(){
        entrada = entradaAnalogica(nControlador);
        //Calculo parametro a
        a = Kp * (1 + T / Ti);
        //Calculo el error
        ek = R - entrada;
        //Calculo el valor de la actuacion actual
        uk = uk1 + a * ek - Kp * ek;
        //Guardo el valor para la siguiente ejecucion
        uk1 = uk;
        salidaAnalogica(nControlador,uk);
    }
    double *peticionDatos(){
        envio[0] = uk;
        envio[1] = entradaAnalogica(nControlador);
        return envio;
    }
    int actuacionOk(){
        return consignaOk && parametrosOk;
    }
    double obtenerT(){
        return T;
    }
};
//Crear los 5 controladores de forma global
Controlador PI[5] = {0, 1, 2, 3, 4};

//hilo de comunicaciones
void comunicaciones(){
    establecePrioridad(2);
    //Crear el canal de comunicaciones
    Canal canal;
    //mensaje como max 5 bytes
    uint8_t paquete[5];
    while(1){
        //Espera a un mensaje y lo guarda en paquete
        canal.recibe(paquete);
        //Guarda el tipo de mensaje
        int tipoMsj = paquete[0];
        //Guarda el contorlador al que hace referencia
        int nControlador = paquete[1];

        switch (tipoMsj) {
        case 0:
            //Paso los parametros al controlador nControlador
            PI[nControlador].recibeParametros(paquete[2], paquete[3],paquete[4]);
            break;
        case 1:
            //Le paso la consigna
            PI[nControlador].recibeConsigna(paquete[2]);
            break;
        case 2:
            //Envio los parametros pedidos
            canal.envia(PI[nControlador].peticionDatos(), sizeof(PI[nControlador].peticionDatos()));
            break;
        }
    }
}
//Hilo para Controlador 0
void PI0(){
    establecePrioridad(1);
    //Espera a que el PI 0 se pueda actuar
    while(!PI[0].actuacionOk()){
        salidaAnalogica(0,0.0);
    }
    double T = PI[0].obtenerT();
    Temporizador t0(T*1000);
    while(1){
        t0.espera();
        PI[1].actuacion();
    }
}
//Hilo para Controlador 1
void PI1(){
    //Espera a que el PI 1 se pueda actuar
    while(!PI[1].actuacionOk()){
        salidaAnalogica(1,0.0);
    }
    double T = PI[1].obtenerT();
    Temporizador t1(T*1000);
    while(1){
        t1.espera();
        PI[1].actuacion();
    }
}
//Hilo para Controlador 2
void PI2(){
    establecePrioridad(1);
    //Espera a que el PI 2 se pueda actuar
    while(!PI[2].actuacionOk()){
        salidaAnalogica(2,0.0);
    }
    double T = PI[2].obtenerT();
    Temporizador t2(T*1000);
    while(1){
        t2.espera();
        PI[1].actuacion();
    }
}
//Hilo para Controlador 3
void PI3(){
    establecePrioridad(1);
    //Espera a que el PI 3 se pueda actuar
    while(!PI[3].actuacionOk()){
        salidaAnalogica(3,0.0);
    };
    double T = PI[3].obtenerT();
    Temporizador t3(T*1000);
    while(1){
        t3.espera();
        PI[3].actuacion();
    }
}
//Hilo para Controlador 4
void PI4(){
    establecePrioridad(1);
    //Espera a que el PI 4 se pueda actuar
    while(!PI[4].actuacionOk()){
        salidaAnalogica(4,0.0);
    }
    double T = PI[4].obtenerT();
    Temporizador t4(T*1000);
    while(1){
        t4.espera();
        PI[4].actuacion();
    }
}

int main()
{
    while(1){
        //Llamada a los hilos
        comunicaciones();
        PI0();
        PI1();
        PI2();
        PI3();
        PI4();
    }
    return 0;
}
```

