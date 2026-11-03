#pragma once

// Minimal check macro shared by the tests: counts failures and prints where.
#include <cstdio>

inline int g_failures = 0;

#define CHECK(cond, ...)                                            \
    do                                                              \
    {                                                               \
        if (!(cond))                                                \
        {                                                           \
            g_failures++;                                           \
            printf("  FAIL %s:%d: %s ", __FILE__, __LINE__, #cond); \
            printf(__VA_ARGS__);                                    \
            printf("\n");                                           \
        }                                                           \
    } while (0)

inline int FinishTests(const char *name)
{
    printf(g_failures ? "%s: %d checks FAILED\n" : "%s: all checks passed\n", name, g_failures);
    return g_failures ? 1 : 0;
}
