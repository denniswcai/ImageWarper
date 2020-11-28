// DC, For processing time measurement:
#include <sys/time.h>

#define TIMING_START_MEASUREMENT                   \
    struct timeval tm_start, tm_last, tm_now;      \
    int    tm_cnt = 1;                             \
    gettimeofday( &tm_start, NULL );               \
    tm_last = tm_start;                            \
    DEBUG_MESSAGE( "Start Timing.\n" )

#define TIMING_CHECK_ELAPSED                       \
    do{                                            \
        gettimeofday( &tm_now, NULL );             \
        DEBUG_MESSAGE( "Timing [#%d]: Time elapsed:%ld miliseconds. (Total elapsed: %ld mili seconds).\n", tm_cnt++, \
                      (tm_now.tv_sec-tm_last.tv_sec) *1000 + (tm_now.tv_usec-tm_last.tv_usec) /1000,               \
                      (tm_now.tv_sec-tm_start.tv_sec)*1000 + (tm_now.tv_usec-tm_start.tv_usec)/1000 );             \
        tm_last = tm_now;                                                                                          \
    } while(0)  