/*
 * Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "movavg.h"

movavg_t *movavg_create(size_t size){
    movavg_t *movavg = calloc(1, sizeof(movavg_t));
    movavg->size = size;
    movavg->items = calloc(size, sizeof(float));
    movavg->counter = 0;
    return movavg;
}

void movavg_free(movavg_t *mov_avg){
    free(mov_avg);
    mov_avg = NULL;
}

void movavg_push(movavg_t *mov_avg, float value){
    mov_avg->items[mov_avg->counter++ % mov_avg->size] = value;
}

float movavg_calc(movavg_t *mov_avg){
    float sum = 0;
    for (int i = 0; i < mov_avg->size; i++){
        sum += mov_avg->items[i];
    }
    return sum / (float) mov_avg->size;
}