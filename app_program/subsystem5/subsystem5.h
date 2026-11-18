#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN 20

#define S5_SERVO_PIN 12u
#define S5_SERVO_CENTRE_US 1500u /* Trim after aligning the sensor on its mount. */

#if APP_S5_BENCH
EXPORT void subsystem5_bench(void);
#endif

#endif
