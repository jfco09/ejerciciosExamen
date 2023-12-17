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
