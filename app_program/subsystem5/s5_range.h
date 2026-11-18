#ifndef S5_RANGE_H
#define S5_RANGE_H

#include <stdbool.h>

#define S5_MAX_RANGE_MM 4000u
#define S5_OPEN_RANGE 4094u
#define S5_UNKNOWN_RANGE 4095u
#define S5_STOP_MM 150u
#define S5_CLEAR_MM 200u

static inline bool s5_range_measured(unsigned range)
{
    return range <= S5_MAX_RANGE_MM;
}

static inline bool s5_range_available(unsigned range)
{
    return s5_range_measured(range) || range == S5_OPEN_RANGE;
}

static inline unsigned s5_range_from_echo(unsigned pulse_us)
{
    unsigned mm = (pulse_us * 343u + 1000u) / 2000u;
    return pulse_us && s5_range_measured(mm) ? mm : S5_OPEN_RANGE;
}

typedef struct {
    unsigned range_mm:12;
    unsigned first_open:1;
    unsigned blocked:1;
} S5_RANGE_GATE;

/* Confirm open space across two pings before clearing a close return or fault.
 * The caller refreshes the accepted sample's timestamp only on success. */
static inline bool s5_accept_range(S5_RANGE_GATE *gate, unsigned range)
{
    bool open = range == S5_OPEN_RANGE;
    bool confirm = !open || gate->first_open;
    gate->first_open = open;
    if (!confirm) return false;
    gate->range_mm = s5_range_available(range) ? range : S5_UNKNOWN_RANGE;
    gate->blocked = !s5_range_available(range) ||
                    range <= (gate->blocked ? S5_CLEAR_MM : S5_STOP_MM);
    return true;
}

#endif
