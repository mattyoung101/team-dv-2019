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
    return (*(int*)b - *(int*)a);
}