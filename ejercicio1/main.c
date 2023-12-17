#include <stdio.h>
int m[] = { 5, 3, 2, 6, 3, 4, 1, 6, 4, 8 };


int main()
{
    int N = sizeof(m) / sizeof(m[0]);
    int minimo = m[0];
    for(int i = 0; i < N; i++){
        if(m[i] < minimo){
            minimo = m[i];
        }
    }
    printf("Minimo es %d\n", minimo);
    return 0;
}
