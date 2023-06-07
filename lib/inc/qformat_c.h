#ifdef __cplusplus
extern "C" {
#endif
#ifndef Q_FORMAT_C_H
#define Q_FORMAT_C_H

#include "stdint.h"

#define DEFAULT_M 15
#define DEFAULT_N 16

#define Q_FORMAT_ONE ((int64_t)(1 << DEFAULT_N))

typedef long long q_format_c;

q_format_c q_format_add(q_format_c A, q_format_c B);
q_format_c q_format_sub(q_format_c A, q_format_c B);
q_format_c q_format_mult(q_format_c A, q_format_c B);
q_format_c q_format_div(q_format_c A, q_format_c B);
q_format_c to_q_format(double value);
double to_double(q_format_c value);
#endif
#ifdef __cplusplus
}
#endif