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
