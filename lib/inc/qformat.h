#ifndef QFORMAT_H
#define QFORMAT_H

#include <iostream>
#include "stdint.h"


class q_format{
private:
    static constexpr int default_m_size = 15;
    static constexpr int default_n_size = 16;
    int m;
    int n;
    int64_t value;

public:
    static constexpr int init_q_format_flag = 0;
    static constexpr int init_double_format_flag = 1;

    q_format() {m = q_format::default_m_size; n = q_format::default_n_size; value = 0;};
    q_format(int m, int n, double value, int input_type = init_double_format_flag);
    q_format(double value, int input_type = init_double_format_flag);

    int64_t get_value() const {
        return this->value;
    }
    int get_m() const {
        return this->m;
    }
    int get_n() const {
        return this->n;
    }
    void set_value(int64_t value) {this->value = value;}

    q_format abs();
    // q_format sqrt();

    q_format &operator=(const q_format &other);
    q_format &operator=(const double other);
    // operator double() const;

    // q_format &operator==(const double other);
    bool operator==(const double other);

    q_format operator+(const q_format &rhs) const;
    q_format operator+(const double rhs);
    friend q_format operator+(double lhs, const q_format& rhs);

    q_format operator-(const q_format &rhs) const;
    q_format operator-(const double rhs);
    friend q_format operator-(double lhs, const q_format& rhs);

    q_format operator/(const q_format &rhs) const;
    q_format operator/(const double rhs);
    friend q_format operator/(double lhs, const q_format& rhs);

    q_format operator*(const q_format &rhs) const;
    q_format operator*(const double rhs);
    friend q_format operator*(double lhs, const q_format& rhs);

    q_format operator+=(const q_format &rhs);
    q_format operator+=(const double rhs);

    q_format operator-=(const q_format &rhs);
    q_format operator-=(const double rhs);

    bool operator<(const q_format &rhs);
    friend bool operator<(double lhs, const q_format& rhs);

    bool operator>(const q_format &rhs);
    friend bool operator>(double lhs, const q_format& rhs);
    

    double to_double() const;
};





#endif