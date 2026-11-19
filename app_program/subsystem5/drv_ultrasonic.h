#ifndef S5_DRV_ULTRASONIC_H
#define S5_DRV_ULTRASONIC_H

#include "s5_range.h"

bool s5_hw_init(void);
void s5_hw_echo_irq(void);
void s5_hw_angle(unsigned angle);
S5_PING s5_hw_ping(void);

#endif
