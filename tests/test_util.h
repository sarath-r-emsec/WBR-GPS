#pragma once

#include <cmath>
#include <cstdio>
#include <string>

static int n_pass = 0;
static int n_fail = 0;

#define ASSERT_TRUE(x)                                                              \
    do {                                                                            \
        if (!(x)) {                                                                 \
            fprintf(stderr, "FAIL %s:%d: %s is false\n", __FILE__, __LINE__, #x);   \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define ASSERT_FALSE(x) ASSERT_TRUE(!(x))

#define ASSERT_EQ(a, b)                                                             \
    do {                                                                            \
        auto _a = (a);                                                              \
        auto _b = (b);                                                              \
        if (!(_a == _b)) {                                                          \
            fprintf(stderr, "FAIL %s:%d: %s != %s\n", __FILE__, __LINE__, #a, #b);  \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define ASSERT_STREQ(a, b)                                                          \
    do {                                                                            \
        if (std::string(a) != std::string(b)) {                                     \
            fprintf(stderr, "FAIL %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__,   \
                    std::string(a).c_str(), std::string(b).c_str());                \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define ASSERT_NEAR(a, b, tol)                                                      \
    do {                                                                            \
        double _a = (double)(a);                                                    \
        double _b = (double)(b);                                                    \
        if (std::fabs(_a - _b) > (tol)) {                                           \
            fprintf(stderr, "FAIL %s:%d: %s (%.9f) != %s (%.9f)\n",                 \
                    __FILE__, __LINE__, #a, _a, #b, _b);                            \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define TEST_MAIN                                                                   \
    int main() {                                                                    \
        run_tests();                                                                \
        printf("%d passed, %d failed\n", n_pass, n_fail);                           \
        return n_fail == 0 ? 0 : 1;                                                 \
    }
