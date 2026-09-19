/* test_util.h - a test harness small enough to read in one sitting. */

#ifndef ROVER_TEST_UTIL_H
#define ROVER_TEST_UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int g_tests_run;
extern int g_tests_failed;
extern const char *g_current_test;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
            g_tests_failed++;                                                  \
            return;                                                            \
        }                                                                      \
    } while (0)

#define CHECK_EQ_INT(actual, expected)                                         \
    do {                                                                       \
        long long _a = (long long)(actual);                                    \
        long long _e = (long long)(expected);                                  \
        if (_a != _e) {                                                        \
            printf("  FAIL %s:%d  %s == %lld, expected %lld\n",                \
                   __FILE__, __LINE__, #actual, _a, _e);                       \
            g_tests_failed++;                                                  \
            return;                                                            \
        }                                                                      \
    } while (0)

#define CHECK_EQ_MEM(actual, expected, len)                                    \
    do {                                                                       \
        if (memcmp((actual), (expected), (len)) != 0) {                        \
            printf("  FAIL %s:%d  %s differs\n    got     ",                   \
                   __FILE__, __LINE__, #actual);                               \
            for (size_t _i = 0; _i < (size_t)(len); _i++)                      \
                printf("%02X", ((const unsigned char *)(actual))[_i]);         \
            printf("\n    expect  ");                                          \
            for (size_t _i = 0; _i < (size_t)(len); _i++)                      \
                printf("%02X", ((const unsigned char *)(expected))[_i]);       \
            printf("\n");                                                      \
            g_tests_failed++;                                                  \
            return;                                                            \
        }                                                                      \
    } while (0)

#define RUN(fn)                                                                \
    do {                                                                       \
        g_tests_run++;                                                         \
        g_current_test = #fn;                                                  \
        int _before = g_tests_failed;                                          \
        fn();                                                                  \
        printf("%-4s %s\n", (g_tests_failed == _before) ? "ok" : "FAIL", #fn); \
    } while (0)

#define TEST_MAIN_EPILOGUE()                                                   \
    do {                                                                       \
        printf("\n%d tests, %d failed\n", g_tests_run, g_tests_failed);        \
        return g_tests_failed == 0 ? 0 : 1;                                    \
    } while (0)

#endif /* ROVER_TEST_UTIL_H */
