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
