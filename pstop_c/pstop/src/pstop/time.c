
#include <time.h>

#include "pstop/time.h"

uint64_t
time_get_now(void)
{
#ifdef __linux__
    struct timespec ts;
    timespec_get(&ts, CLOCK_REALTIME);

    // Calculate milliseconds
    return ((uint64_t)ts.tv_sec) * 1000ULL + ((uint64_t)ts.tv_nsec) / 1000000ULL;
#else
    return 0U;
#endif
}
