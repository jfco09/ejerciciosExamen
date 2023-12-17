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
