#include <qformat.h>
#include <math.h>

q_format::q_format(int m, int n, double value, int input_type)
{
    this->m = m;
    this->n = n;
    if (input_type == q_format::init_double_format_flag) {
        int64_t max_numerator = pow(2,m) - 1;
        if (value > max_numerator) {
            return;
        }
        // this->value = (uint64_t)(value) << n;
        this->value = (uint64_t)((value) * pow(2,n));
    } else {
        this->value = (uint64_t)value;
    }
    return;
}

q_format::q_format(double value, int input_type)
{
    *this = q_format(q_format::default_m_size, q_format::default_n_size, value, input_type);
}

q_format q_format::abs()
{
    if (this->value < 0) {
        return q_format(this->m, this->m, -this->value, q_format::init_q_format_flag);
    }
    return *this;
}

q_format &q_format::operator=(const q_format &other)
{
    this->m = other.m;
    this->n = other.n;
    this->value = other.value;
    return *this;
}
q_format &q_format::operator=(const double other)
{
    *this = q_format(this->m, this->n, other, q_format::init_double_format_flag);
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
    int64_t temp = this->value + rhs.get_value();
    q_format result(this->m, this->n, temp, q_format::init_q_format_flag);
    return result;
}
q_format q_format::operator+(const double rhs)
{
    q_format temp(this->m, this->n, rhs);
    return *this + temp;
}

q_format operator+(double lhs, const q_format& rhs)
{
    q_format temp(rhs.get_m(), rhs.get_n(), lhs);
    return temp + rhs;
}

q_format q_format::operator-(const q_format &rhs) const
{
    q_format result(this->m, this->n, this->value - rhs.get_value() ,q_format::init_q_format_flag);
    return result;
}
q_format q_format::operator-(const double rhs)
{
    q_format temp(this->m, this->n, rhs);
    return *this - temp;
}
q_format operator-(double lhs, const q_format& rhs)
{
    q_format temp(rhs.get_m(), rhs.get_n(), lhs);
    return temp - rhs;
}

q_format q_format::operator/(const q_format &rhs) const
{
    // int64_t temp1 = rhs.get_value() >> this->n; // TODO8
    // int64_t temp = (this->value / rhs.get_value()) >> this->n;;
    // q_format result(this->m, this->n, temp ,q_format::init_q_format_flag);
    // return result;

    int64_t temp = (int64_t)this->value << this->n;
    /* Rounding: mid values are rounded up (down for negative values). */
    /* OR compare most significant bits i.e. if (((temp >> 31) & 1) == ((b >> 15) & 1)) */
    // if ((temp >= 0 && rhs.get_value() >= 0) || (temp < 0 && rhs.get_value() < 0)) {   
    //     temp += rhs.get_value() >> 1;    /* OR shift 1 bit i.e. temp += (b >> 1); */
    // } else {
    //     temp -= rhs.get_value() >> 1;    /* OR shift 1 bit i.e. temp -= (b >> 1); */
    // }
    q_format result(this->m, this->n, temp / rhs.get_value() ,q_format::init_q_format_flag);
    return result;
}
q_format q_format::operator/(const double rhs)
{
    q_format temp(this->m, this->n, rhs);
    return *this / temp;
}
q_format operator/(double lhs, const q_format& rhs)
{
    q_format temp(rhs.get_m(), rhs.get_n(), lhs);
    return temp / rhs;
}

q_format q_format::operator*(const q_format &rhs) const
{
    // int64_t temp1 = this->value >> this->n;
    // int64_t temp2 = rhs.get_value() >> this->n; // TODO
    // int64_t temp3 = (temp1 * temp2) << this->n;

    int64_t temp1 = this->value * rhs.get_value();
    temp1 += 1 << (this->n-1);
    temp1 >>= this->n;

    q_format result(this->m, this->n, temp1 ,q_format::init_q_format_flag);
    return result;
}
q_format q_format::operator*(const double rhs)
{
    q_format temp(this->m, this->n, rhs);
    return *this * temp;
}
q_format operator*(double lhs, const q_format& rhs)
{
    q_format temp(rhs.get_m(), rhs.get_n(), lhs);
    return temp * rhs;
}

q_format q_format::operator+=(const q_format &rhs)
{
    *this = *this + rhs;
    return *this + rhs;
}
q_format q_format::operator+=(const double rhs)
{
    *this = *this + rhs;
    return *this + rhs;
}

q_format q_format::operator-=(const q_format &rhs)
{
    *this = *this - rhs;
    return *this - rhs;
}
q_format q_format::operator-=(const double rhs)
{
    *this = *this - rhs;
    return *this - rhs;
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
    return ((double)(temp)) / pow(2, this->n);
}
