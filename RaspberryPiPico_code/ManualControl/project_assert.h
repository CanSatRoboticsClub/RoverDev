// project-wide assertion macro

#ifndef PROJECT_ASSERT_H
#define PROJECT_ASSERT_H

#include <stdio.h>
#include <stdbool.h>

#define c_assert(e) ((e) ? (true) : (printf("%s,%d: assertion '%s' failed\n", \
    __FILE__, __LINE__, #e), false))

#endif