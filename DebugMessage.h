//Ver 1.1, DC 2020/10/16
#include <stdio.h>

#ifndef __DEBUG_MESSAGE_H__
#define __DEBUG_MESSAGE_H__

#define DEBUG_MESSAGE(x, ...)   printf(x, ##__VA_ARGS__ )
#define DEBUG_WARNING(x, ...)   printf(x, ##__VA_ARGS__ )
#define DEBUG_ENTER_FUNCTION    printf(">> In %s()\n", __FUNCTION__ )
#define DEBUG_EXIT_FUNCTION     printf("<< Out %s()\n", __FUNCTION__ )
#define DEBUG_ASSERT( a, m )    if( !(a) ) { printf("ASSERTION ERROR:"); printf(m); printf("\n"); }
#define DEBUG_CHECK_RESULT( result, x, ... )                                                                   \
        if( (result) != 0 )                                                                                    \
        {   printf("ERROR!: ");                                                                                \
            printf( x, ##__VA_ARGS__ );                                                                        \
            printf( "\n        (Error Code=%d, at line %d, in function \"%s()\")\n", (result), __LINE__, __FUNCTION__ );   \
            exit(result);                                                                                      \
        }

#define ERROR_MESSAGE(x, ...)   do { printf("ERROR: "); printf(x, ##__VA_ARGS__ ); printf("\n"); } while(0)
#define PRINT_MESSAGE(x, ...)   printf(x, ##__VA_ARGS__ )

#endif
