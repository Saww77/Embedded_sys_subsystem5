#ifndef S5_DRV_ULTRASONIC_H
#define S5_DRV_ULTRASONIC_H

void s5_hw_init(void);
void s5_hw_angle(unsigned angle);
unsigned s5_hw_measure_mm(void);

#endif
