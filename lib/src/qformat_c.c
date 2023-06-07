#include "qformat_c.h"
#include "math.h"



q_format_c q_format_add(q_format_c A, q_format_c B)
{
    return A + B;
}

q_format_c q_format_sub(q_format_c A, q_format_c B)
{
    return A - B;
}

q_format_c q_format_mult(q_format_c A, q_format_c B)
{
    int64_t temp1 = A * B;
    // temp1 += 1 << (DEFAULT_N);
    temp1 >>= DEFAULT_N;
    return temp1;
}

q_format_c q_format_div(q_format_c A, q_format_c B)
{
    int64_t temp = (int64_t)A << DEFAULT_N;
    return temp / B;
}

q_format_c to_q_format(double value)
{
    return (uint64_t)((value) * (Q_FORMAT_ONE));
}

double to_double(q_format_c value)
{
    return ((double)(value)) * pow(2, -DEFAULT_N);
}