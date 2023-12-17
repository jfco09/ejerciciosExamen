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
