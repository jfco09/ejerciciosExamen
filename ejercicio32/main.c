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
