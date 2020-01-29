/*
 * Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <stdint.h>
#include <stdlib.h>

// Moving average

typedef struct {
    size_t size;
    uint16_t counter;
    float *items;
} movavg_t;

/** Instantiates a new moving average object **/
movavg_t *movavg_create(size_t size);
/** Frees a moving average object **/
void movavg_free(movavg_t *mov_avg);
/** Pushes a new value to the moving average items array, looping around to the start if required **/
void movavg_push(movavg_t *mov_avg, float value);
/** Calculates the moving average **/
float movavg_calc(movavg_t *mov_avg);