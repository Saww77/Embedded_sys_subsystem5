#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN 20

#define S5_SERVO_PIN 12u

#if APP_S5_BENCH
EXPORT void subsystem5_bench(void);
#endif

#endif
