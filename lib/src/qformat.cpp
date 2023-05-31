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
        this->value = (uint64_t)(value) << n;
    } else {
        this->value = value;
    }
    return;
}

q_format::q_format(double value, int input_type)
{
    *this = q_format(q_format::default_m_size, q_format::default_n_size, value, input_type);
}

q_format &q_format::operator=(const q_format &other)
{
    this->m = other.m;
    this->n = other.n;
    this->value = other.value;
    return *this;
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
    int64_t temp1 = rhs.get_value() >> this->n; // TODO
    int64_t temp = this->value / temp1;
    q_format result(this->m, this->n, temp ,q_format::init_q_format_flag);
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
    int64_t temp1 = this->value >> this->n;
    int64_t temp2 = rhs.get_value() >> this->n; // TODO
    int64_t temp3 = (temp1 * temp2) << this->n;

    q_format result(this->m, this->n, temp3 ,q_format::init_q_format_flag);
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


double q_format::change_double()
{
    return double(this->value) / pow(2, this->n);
}
