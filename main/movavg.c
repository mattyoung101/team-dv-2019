#include "movavg.h"

mov_avg_t *mov_avg_create(size_t size){
    mov_avg_t *movavg = calloc(1, sizeof(mov_avg_t));
    movavg->size = size;
    movavg->items = calloc(size, sizeof(float));
    movavg->counter = 0;
    return movavg;
}

void mov_avg_push(mov_avg_t *mov_avg, float value){
    mov_avg->items[mov_avg->counter++ % mov_avg->size] = value;
}

float mov_avg_calc(mov_avg_t *mov_avg){
    float sum = 0;
    for (int i = 0; i < mov_avg->size; i++){
        sum += mov_avg->items[i];
    }
    return sum / (float) mov_avg->size;
}