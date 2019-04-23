#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include <math.h>
#include "defines.h"
#include "utils.h"
#include "mplexer.h"
#include "esp_timer.h"
#include <inttypes.h>
#include "movavg.h"

extern float tsopAngle;
extern float tsopStrength;
extern float tsopAvgAngle;
extern float tsopAvgStrength;

/** Initialises the TSOP pins and multiplexer **/
void tsop_init(void);
/** Reads all the TSOP values into the temp array **/
void tsop_update(void *args);
/** Calculates tsopAngle and tsopStrength for the best n values **/
void tsop_calc(void);
/** Dumps the temp array to UART **/
void tsop_dump(void);