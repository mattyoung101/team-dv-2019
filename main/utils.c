#include "utils.h"

int32_t mod(int32_t x, int32_t m){
    int16_t r = x % m;
    return r < 0 ? r + m : r;
}

float floatMod(float x, float m) {
    float r = fmodf(x, m);
    return r < 0 ? r + m : r;
}

int number_comparator_descending(const void *a, const void *b){
    // descending order so b - a
    return (*(int*)b - *(int*)a);
}

float angleBetween(float angleCounterClockwise, float angleClockwise){
    return mod(angleClockwise - angleCounterClockwise, 360);
}

float midAngleBetween(float angleCounterClockwise, float angleClockwise){
    float ang = angleBetween(angleCounterClockwise, angleClockwise);
    return fminf(ang, 360 - ang);
}