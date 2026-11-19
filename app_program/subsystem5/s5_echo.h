#ifndef S5_ECHO_H
#define S5_ECHO_H

#include "s5_range.h"
#include <stdint.h>

enum { S5_CAPTURE_IDLE, S5_CAPTURE_RISE, S5_CAPTURE_FALL, S5_CAPTURE_DONE, S5_CAPTURE_BAD };

typedef struct {
    uint32_t triggered_at, rose_at;
    unsigned state : 3;
    unsigned pulse_us : 15;
} S5_ECHO_CAPTURE;

/* The task arms and drains this state with the local interrupt masked. */
static inline void s5_capture_edge(volatile S5_ECHO_CAPTURE *capture, uint32_t now,
                                   bool rise, bool fall)
{
    if (now - capture->triggered_at >= S5_ECHO_TIMEOUT_US) return;
    if (capture->state == S5_CAPTURE_RISE) {
        if (rise && !fall) {
            capture->rose_at = now;
            capture->state = S5_CAPTURE_FALL;
        } else {
            capture->state = S5_CAPTURE_BAD;
        }
    } else if (capture->state == S5_CAPTURE_FALL) {
        if (fall && !rise) {
            capture->pulse_us = now - capture->rose_at;
            capture->state = S5_CAPTURE_DONE;
        } else {
            capture->state = S5_CAPTURE_BAD;
        }
    }
}

#endif
