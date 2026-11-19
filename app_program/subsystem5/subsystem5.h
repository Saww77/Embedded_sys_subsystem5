#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN 20

#define S5_SERVO_PIN 12u
/* Calibrate with the sensor mounted. At centre it faces straight ahead.
 * Starting values reproduce the original PWM commands. */
#define S5_SERVO_RIGHT_US 1167u  /* 30 degrees */
#define S5_SERVO_CENTRE_US 1500u /* 90 degrees */
#define S5_SERVO_LEFT_US 1833u   /* 150 degrees */

void subsystem5_request_scan(void);

#if APP_S5_BENCH
EXPORT void subsystem5_bench(void);
#endif
#if APP_S5_SENSOR_TEST
EXPORT void subsystem5_sensor_test(void);
#endif

#endif
