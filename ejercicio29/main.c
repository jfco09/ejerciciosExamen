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
