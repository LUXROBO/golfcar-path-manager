#include <qformat.h>
#include <math.h>

q_format::q_format(double value, int input_type)
{
    if (input_type == q_format::init_double_format_flag) {
        // int64_t max_numerator = pow(2,m) - 1;
        // if (value > max_numerator) {
        //     return;
        // }
        // // this->value = (uint64_t)(value) << n;
        // this->value = (uint64_t)((value) * pow(2,n));
        this->value = (uint64_t)((value) * ((int64_t)1 << q_format::default_n_size));
    } else {
        this->value = (uint64_t)value;
    }
    return;
}
q_format q_format::abs()
{
    if (this->value < 0) {
        return q_format(-this->value, q_format::init_q_format_flag);
    }
    return *this;
}

// q_format &q_format::operator=(const q_format &other)
// {
//     this->m = other.m;
//     this->n = other.n;
//     this->value = other.value;
//     return *this;
// }
q_format &q_format::operator=(const double other)
{
    *this = q_format(other, q_format::init_double_format_flag);
    return *this;
}

bool q_format::operator==(const double other)
{
    if (this->to_double() == other) {
        return true;
    }
    return false;
}

q_format q_format::operator+(const q_format &rhs) const
{
    // int64_t temp = this->value + rhs.get_value();
    // q_format result(this->m, this->n, this->value + rhs.get_value(), q_format::init_q_format_flag);
    return q_format(this->value + rhs.get_value(), q_format::init_q_format_flag);
}
q_format q_format::operator+(const double rhs)
{
    q_format temp(rhs);
    return *this + temp;
}

q_format operator+(double lhs, const q_format& rhs)
{
    q_format temp(lhs);
    return temp + rhs;
}

q_format q_format::operator-(const q_format &rhs) const
{
    // q_format result(this->m, this->n, this->value - rhs.get_value() ,q_format::init_q_format_flag);
    return q_format(this->value - rhs.get_value() ,q_format::init_q_format_flag);;
}
q_format q_format::operator-(const double rhs)
{
    q_format temp(rhs);
    return *this - temp;
}
q_format operator-(double lhs, const q_format& rhs)
{
    q_format temp(lhs);
    return temp - rhs;
}

q_format q_format::operator/(const q_format &rhs) const
{
    // int64_t temp1 = rhs.get_value() >> this->n; // TODO8
    // int64_t temp = (this->value / rhs.get_value()) >> this->n;;
    // q_format result(this->m, this->n, temp ,q_format::init_q_format_flag);
    // return result;

    int64_t temp = (int64_t)this->value << q_format::default_n_size;
    /* Rounding: mid values are rounded up (down for negative values). */
    /* OR compare most significant bits i.e. if (((temp >> 31) & 1) == ((b >> 15) & 1)) */
    // if ((temp >= 0 && rhs.get_value() >= 0) || (temp < 0 && rhs.get_value() < 0)) {
    //     temp += rhs.get_value() >> 1;    /* OR shift 1 bit i.e. temp += (b >> 1); */
    // } else {
    //     temp -= rhs.get_value() >> 1;    /* OR shift 1 bit i.e. temp -= (b >> 1); */
    // }
    // q_format result(this->m, this->n, temp / rhs.get_value() ,q_format::init_q_format_flag);
    return q_format(temp / rhs.get_value() ,q_format::init_q_format_flag);;
}
q_format q_format::operator/(const double rhs)
{
    q_format temp(rhs);
    return *this / temp;
}
q_format operator/(double lhs, const q_format& rhs)
{
    q_format temp(lhs);
    return temp / rhs;
}

q_format q_format::operator*(const q_format &rhs) const
{
    // int64_t temp1 = this->value >> this->n;
    // int64_t temp2 = rhs.get_value() >> this->n; // TODO
    // int64_t temp3 = (temp1 * temp2) << this->n;

    int64_t temp1 = this->value * rhs.get_value();
    temp1 += 1 << (q_format::default_n_size-1);
    temp1 >>= q_format::default_n_size;

    // q_format result(this->m, this->n, temp1 ,q_format::init_q_format_flag);
    return q_format(temp1 ,q_format::init_q_format_flag);
}
q_format q_format::operator*(const double rhs)
{
    q_format temp(rhs);
    return *this * temp;
}
q_format operator*(double lhs, const q_format& rhs)
{
    q_format temp(lhs);
    return temp * rhs;
}

q_format q_format::operator+=(const q_format &rhs)
{
    *this = *this + rhs;
    return *this;
}
q_format q_format::operator+=(const double rhs)
{
    *this = *this + rhs;
    return *this;
}

q_format q_format::operator-=(const q_format &rhs)
{
    *this = *this - rhs;
    return *this;
}
q_format q_format::operator-=(const double rhs)
{
    *this = *this - rhs;
    return *this;
}

bool q_format::operator<(const q_format &rhs)
{
    return this->value < rhs.get_value();
}
bool operator<(double lhs, const q_format& rhs)
{
    return lhs < rhs.get_value();
}

bool q_format::operator>(const q_format &rhs)
{
    return this->value > rhs.get_value();
}
bool operator>(double lhs, const q_format& rhs)
{
    return lhs > rhs.get_value();
}

double q_format::to_double() const
{
    int64_t temp = this->value;
    return ((double)(temp)) * pow(2, -q_format::default_n_size);
}
