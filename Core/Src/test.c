#include "test.h"
#if defined(__CC_ARM)
#pragma arm section code = ".qspi"
#pragma no_inline
static char test_string[] = 
#elif defined(__ICCARM__)
static char test_string[] @ ".qspi" =
#elif defined(__GNUC__)
static char __attribute__((section(".qspi"))) test_string[] = 
#endif
"i am the test executing scripting\
welcome to the evlution";

const test_size = sizeof(test_string) / sizeof(test_string[0]);

char get_test_char(uint32_t index)
{
    if (index >= test_size) {
        return ' ';
    }
    return test_string[index];
}

#if defined(__CC_ARM)
#pragma arm section code
#endif