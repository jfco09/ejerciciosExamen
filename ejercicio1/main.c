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
