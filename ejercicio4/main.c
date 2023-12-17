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
