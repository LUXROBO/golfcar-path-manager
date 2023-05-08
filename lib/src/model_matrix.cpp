#include "model_matrix.h"

#include <cmath>


static q15_t q15_division(q15_t a, q15_t b)
{
    /* pre-multiply by the base (Upscale to Q16 so that the result will be in Q8 format) */
    q31_t temp = (q31_t)a << 15;
    /* Rounding: mid values are rounded up (down for negative values). */
    /* OR compare most significant bits i.e. if (((temp >> 31) & 1) == ((b >> 15) & 1)) */
    if ((temp >= 0 && b >= 0) || (temp < 0 && b < 0)) {
        temp += b / 2;    /* OR shift 1 bit i.e. temp += (b >> 1); */
    } else {
        temp -= b / 2;    /* OR shift 1 bit i.e. temp -= (b >> 1); */
    }
    return (q15_t)(temp / b);

}

// ModelMatrix
ModelMatrix::ModelMatrix()
    : row_(4), column_(4) {
    q15_t temp = 0;
    arm_fill_q15(temp, element_, 16);
    // element_.resize(row_ * column_);
}

ModelMatrix::ModelMatrix(const ModelMatrix &other)
    : row_(other.row()), column_(other.column()) {
    arm_copy_q15(other.element(), element_, 16);
    // element_ = other.element();
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column)
    : row_(row), column_(column) {
    // element_.resize(row_ * column_);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const q15_t *element)
    : row_(row), column_(column) {
    arm_copy_q15(element, element_, 16);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const q15_t **element)
    : row_(row), column_(column) {

    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r][c];
        }
    }
}

// ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const std::vector<q15_t> element)
//     : row_(row), column_(column) {
//     for (unsigned int r = 0; r < row; r++) {
//         for (unsigned int c = 0; c < column; c++) {
//             element_[r * column + c] = element[r * column + c];
//         }
//     }
// }

ModelMatrix::~ModelMatrix() {
    memset((void*)element_, 0 ,sizeof(q15_t) * 16);
}

unsigned int ModelMatrix::row() const {
    return row_;
}

unsigned int ModelMatrix::column() const {
    return column_;
}

q15_t* ModelMatrix::element() const {
    return (q15_t* )element_;
}

q15_t ModelMatrix::get(const unsigned int row, const unsigned int column) const {
    if (row > row_) {
        return 0.0;
    } else if (column > column_) {
        return 0.0;
    } else {
        return element_[row * column_ + column];
    }
}

void ModelMatrix::set(const unsigned int row, const unsigned int column, const q15_t value) {
    if (row > row_) {
        return;
    } else if (column > column_) {
        return;
    } else {
        element_[row * column_ + column] = value;
    }
}

ModelMatrix ModelMatrix::zero(const unsigned int row, const unsigned int column) {
    return ModelMatrix(row, column);
}

ModelMatrix ModelMatrix::one(const unsigned int row, const unsigned int column) {
    q15_t mat[16] = {0, };
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            mat[r * column + c] = 1.0;
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::identity(const unsigned int row, const unsigned int column) {
    q15_t mat[16] = {0, };
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            if (r == c) {
                mat[r * column + c] = 1.0;
            } else {
                mat[r * column + c] = 0.0;
            }
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::transpose() {
    q15_t ele[16];
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            ele[c * row_ + r] = element_[r * column_ + c];
        }
    }
    return ModelMatrix(column_, row_, ele);
}

q15_t ModelMatrix::determinant() {
    if (row_ == column_) {
        return determinant(element_, row_);
    } else if (row_ > column_) {
        return determinant((this->transpose() * (*this)).element(), column_);
    } else {
        return determinant(((*this) * this->transpose()).element(), row_);
    }
}

ModelMatrix ModelMatrix::inverse() {
    if (row_ == column_) {
        // square matrix
        q15_t result[16];
        matrixInversion(element_, result, row_);
        return ModelMatrix(row_, column_, result);
    } else {
        // rectangular matrix
        return pseudoInverse();
    }
}

ModelMatrix ModelMatrix::inverse(const q15_t sigma) {
    if (row_ <= column_) {
        // m by n matrix (n >= m)
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return this->transpose() * ((*this) * (this->transpose()) + temp).inverse();
    } else {
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return (this->transpose() * (*this) + temp).inverse() * this->transpose();
    }
}

q15_t ModelMatrix::length() const {
    q15_t l = 0;
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            q15_t temp;
            arm_mult_q15(&element_[r * column_ + c], &element_[r * column_ + c], &temp, 1);
            arm_add_q15(&l, &temp, &l, 1);
            // l += element_[r * column_ + c] * element_[r * column_ + c];
        }
    }
    arm_sqrt_q15(l, &l);
    return l;
}

ModelMatrix ModelMatrix::normalize() const {
    q15_t l = length();
    if (l == 0) {
        return ModelMatrix::identity(row_, column_);
    } else {
        //std::vector<float> ele(row_, column_);
        q15_t ele[16] = {0,};
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                ele[r * column_ + c] = q15_division(element_[r * column_ + c], l);
                // ele[r * column_ + c] = element_[r * column_ + c] / l;
            }
        }
        return ModelMatrix(row_, column_, ele);
    }
}

q15_t ModelMatrix::dot(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        q15_t dot = 0.0;
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                q15_t temp;
                arm_mult_q15(&element_[r * column_ + c], &rhs.element()[r * column_ + c], &temp, 1);
                // dot += element_[r * column_ + c] * rhs.element()[r * column_ + c];
            }
        }
        return dot;
    } else {
        return 0.0;
    }
}

ModelMatrix ModelMatrix::cross(const ModelMatrix &rhs) {
    if (row_ == 3 && column_ == 1 && rhs.row() == 3 && rhs.column() == 1) {
        // std::vector<float> ele(3);
        q15_t ele[3];
        q15_t temp[2];
        arm_mult_q15(&element_[1], &rhs.element()[2], &temp[0], 1);
        arm_mult_q15(&element_[2], &rhs.element()[1], &temp[1], 1);
        ele[0] = temp[0] - temp[1];
        arm_mult_q15(&element_[2], &rhs.element()[0], &temp[0], 1);
        arm_mult_q15(&element_[0], &rhs.element()[2], &temp[1], 1);
        ele[1] = temp[0] - temp[1];
        arm_mult_q15(&element_[0], &rhs.element()[1], &temp[0], 1);
        arm_mult_q15(&element_[1], &rhs.element()[0], &temp[1], 1);
        ele[2] = temp[0] - temp[1];
        return ModelMatrix(row_, column_, ele);
    } else {
        return ModelMatrix::zero(3, 1);
    }
}

ModelMatrix ModelMatrix::cross() {
    if (row_ == 3 && column_ == 1) {
        q15_t ele[9];
        ele[0 * 3 + 0] = 0;
        arm_negate_q15(&element_[2], &ele[0 * 3 + 1], 1);
        // ele[0 * 3 + 1] = -element_[2];
        ele[0 * 3 + 2] = element_[1];
        ele[1 * 3 + 0] = element_[2];
        ele[1 * 3 + 1] = 0.0;
        arm_negate_q15(&element_[0], &ele[1 * 3 + 2], 1);
        // ele[1 * 3 + 2] = -element_[0];
        arm_negate_q15(&element_[1], &ele[2 * 3 + 0], 1);
        // ele[2 * 3 + 0] = -element_[1];
        ele[2 * 3 + 1] = element_[0];
        ele[2 * 3 + 2] = 0.0;
        return ModelMatrix(3, 3, ele);
    } else {
        return ModelMatrix::zero(3, 3);
    }
}

ModelMatrix &ModelMatrix::operator=(const ModelMatrix &other) {
    this->row_ = other.row_;
    this->column_ = other.column();
    arm_copy_q15(this->element_, other.element(), 1);
    // this->element_ = other.element();
    return *this;
}

ModelMatrix ModelMatrix::operator+(const q15_t &rhs) {
    q15_t temp[16];
    arm_offset_q15(this->element_, rhs, temp, this->column_ * this->row_);
    // ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
	return ModelMatrix(this->row_, this->column_, temp);
}

ModelMatrix ModelMatrix::operator+(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        // std::vector<float> temp(row_ * column_);
        q15_t temp[16];
        arm_add_q15(this->element_, rhs.element(), temp, this->row_ * this->column_);
        return ModelMatrix(row_, column_, temp);
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator-(const q15_t &rhs) {
    q15_t temp[16];
    q15_t temp2;
    arm_negate_q15(&rhs, &temp2, 1);
    arm_offset_q15(this->element_, temp2, temp, this->column_ * this->row_);
    // ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
	return ModelMatrix(this->row_, this->column_, temp);
}

ModelMatrix ModelMatrix::operator-(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        q15_t temp[16];
        arm_negate_q15(rhs.element(), temp, this->row_ * this->column_);
        arm_add_q15(this->element_, temp, temp, this->row_ * this->column_);
        return ModelMatrix(row_, column_, temp);
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator*(const q15_t &rhs) {
    q15_t temp[16];
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            arm_mult_q15(&element_[r * column_ + c], &rhs, &temp[r * column_ + c], 1);
            // temp[r * column_ + c] = element_[r * column_ + c] * rhs;
        }
    }
    return ModelMatrix(row_, column_, temp);
}

ModelMatrix ModelMatrix::operator*(const ModelMatrix &rhs) {
    if (column_ == rhs.row()) {
        q15_t temp[16];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < rhs.column(); c++) {
                temp[r * rhs.column() + c] = 0;
                for (unsigned int k = 0; k < column_; k++) {
                    arm_mult_q15(&element_[r * column_ + k], &rhs.element()[k * rhs.column() + c], &temp[r * rhs.column() + c], 1);
                }
            }
        }
        return ModelMatrix(row_, rhs.column(), temp);
    } else {
		return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix operator+(const q15_t &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row(), rhs.column()) * lhs;
    return left + rhs;
}

ModelMatrix operator-(const q15_t &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row(), rhs.column()) * lhs;
    return left - rhs;
}

ModelMatrix operator*(const q15_t &lhs, const ModelMatrix &rhs) {
    // std::vector<float> temp(rhs.row() * rhs.column());
    q15_t temp[16];
    for (unsigned int r = 0; r < rhs.row(); r++) {
        for (unsigned int c = 0; c < rhs.column(); c++) {
            arm_mult_q15(&rhs.element()[r * rhs.column() + c], &lhs, &temp[r * rhs.column() + c], 1);
            // temp[r * rhs.column() + c] = rhs.element()[r * rhs.column() + c] * lhs;
        }
    }
    return ModelMatrix(rhs.row(), rhs.column(), temp);
}

ModelMatrix ModelMatrix::pseudoInverse() {
    if (row_ == column_) {
        return inverse();
    } else if (row_ > column_) {
        return pseudoInverseL();
    } else {
        return pseudoInverseR();
    }
}

ModelMatrix ModelMatrix::pseudoInverseR() {
    return this->transpose() * ((*this) * (this->transpose())).inverse();
}

ModelMatrix ModelMatrix::pseudoInverseL() {
    return ((this->transpose()) * (*this)).inverse() * this->transpose();
}

q15_t ModelMatrix::determinant(q15_t *matrix, int order) {
    // the determinant value
    q15_t det = 1.0;
    q15_t temp[2];
    // stop the recursion when matrix is a single element
    if (order == 1) {
        det = matrix[0];
    } else if (order == 2) {
        arm_mult_q15(&matrix[0 * 2 + 0], &matrix[1 * 2 + 1], &temp[0], 1);
        arm_mult_q15(&matrix[0 * 2 + 0], &matrix[1 * 2 + 1], &temp[1], 1);
        det = temp[0] - temp[1];
        // det = matrix[0 * 2 + 0] * matrix[1 * 2 + 1] - matrix[0 * 2 + 1] * matrix[1 * 2 + 0];
    } else if (order == 3) {
        arm_mult_q15(&matrix[0 * 3 + 0], &matrix[1 * 3 + 1], &temp[0], 1);
        arm_mult_q15(&temp[0], &matrix[2 * 3 + 2], &temp[1], 1);
        arm_add_q15(&temp[1], &det, &det, 1);

        arm_mult_q15(&matrix[0 * 3 + 1], &matrix[1 * 3 + 2], &temp[0], 1);
        arm_mult_q15(&temp[0], &matrix[2 * 3 + 0], &temp[1], 1);
        arm_add_q15(&temp[1], &det, &det, 1);

        arm_mult_q15(&matrix[0 * 3 + 2], &matrix[1 * 3 + 0], &temp[0], 1);
        arm_mult_q15(&temp[0], &matrix[2 * 3 + 1], &temp[1], 1);
        arm_add_q15(&temp[1], &det, &det, 1);

        arm_mult_q15(&matrix[0 * 3 + 0], &matrix[1 * 3 + 2], &temp[0], 1);
        arm_mult_q15(&temp[0], &matrix[2 * 3 + 1], &temp[1], 1);
        det -= temp[1];

        arm_mult_q15(&matrix[0 * 3 + 1], &matrix[1 * 3 + 0], &temp[0], 1);
        arm_mult_q15(&temp[0], &matrix[2 * 3 + 2], &temp[1], 1);
        det -= temp[1];

        arm_mult_q15(&matrix[0 * 3 + 2], &matrix[1 * 3 + 1], &temp[0], 1);
        arm_mult_q15(&temp[0], &matrix[2 * 3 + 0], &temp[1], 1);
        det -= temp[1];

        // det = matrix[0 * 3 + 0] * matrix[1 * 3 + 1] * matrix[2 * 3 + 2]
        //     + matrix[0 * 3 + 1] * matrix[1 * 3 + 2] * matrix[2 * 3 + 0]
        //     + matrix[0 * 3 + 2] * matrix[1 * 3 + 0] * matrix[2 * 3 + 1]
        //     - matrix[0 * 3 + 0] * matrix[1 * 3 + 2] * matrix[2 * 3 + 1]
        //     - matrix[0 * 3 + 1] * matrix[1 * 3 + 0] * matrix[2 * 3 + 2]
        //     - matrix[0 * 3 + 2] * matrix[1 * 3 + 1] * matrix[2 * 3 + 0];
    } else {
        // generation of temporary matrix
        // std::vector<float> temp_matrix = matrix;
        q15_t temp_matrix[15];
        arm_copy_q15(temp_matrix, temp_matrix, 16);

        // gaussian elimination
        for (int i = 0; i < order; i++) {
            // find max low
            q15_t temp = 0;
            q15_t temp2;
            int max_row = i;
            for (int j = i; j < order; j++) {
                arm_abs_q15(&temp_matrix[j * order + i], &temp2, 1);
                if (temp2 > temp) {
                    temp = temp2;
                    max_row = j;
                }
            }
            arm_abs_q15(&temp_matrix[max_row * order + i], &temp2, 1);
            float temp_float = 0.0001;
            q15_t temp3;
            arm_float_to_q15(&temp_float, &temp3, 1);
            if (temp2 > temp3) {
                // transfer row
                if (max_row != i) {
                    for (int j = 0; j < order; j++) {
                        temp -= temp_matrix[max_row * order + j];
                        temp_matrix[max_row * order + j] = temp_matrix[i * order + j];
                        temp_matrix[i * order + j] = temp;
                    }
                }
                // elemination
                for (int j = i + 1; j < order; j++) {
                    temp = q15_division(temp_matrix[j * order + i], temp_matrix[i * order + i]);
                    // temp = temp_matrix[j * order + i] / temp_matrix[i * order + i];
                    for (int k = i; k < order; k++) {
                        arm_mult_q15(&temp_matrix[i * order + k], &temp, &temp2, 1);
                        temp_matrix[j * order + k] -= temp2;
                    }
                }
            }
        }

        for (int i = 0; i < order; i++) {
            arm_mult_q15(&temp_matrix[i * order + i], &det, &det, 1);
        }
    }

    return det;
}

void ModelMatrix::matrixInversion(q15_t* matrix, q15_t* result, int order) {
    q15_t matA[16];
    q15_t matB[16];
    arm_copy_q15(matA, matrix, 16);
    arm_copy_q15(matB, ModelMatrix::identity(order, order).element(), order* order);
    q15_t temp[3];

    // Gauss-Jordan
    // Forward
    for (int i = 0; i < order; i++) {
        // max row
        temp[0] = 0.000;
        int max_row = i;
        for (int j = i; j < order; j++) {
            arm_abs_q15(&matA[j * order + i], &temp[1], 1);
            if (temp[1] > temp[0]) {
                temp[0] = temp[1];
                max_row = j;
            }
        }
        // change row
        temp[2] =  matA[max_row * order + i];
        for (int j = 0; j < order; j++) {
            temp[0] = matA[max_row * order + j];
            matA[max_row * order + j] = matA[i * order + j];
            matA[i * order + j] = q15_division(temp[0], temp[2]);

            temp[0] = matB[max_row * order + j];
            matB[max_row * order + j] = matB[i * order + j];
            matB[i * order + j] = q15_division(temp[0], temp[2]);
        }
        for (int j = i + 1; j < order; j++) {
            temp[0] = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                arm_mult_q15(&matA[i * order + k], &temp[0], &temp[1], 1);
                matA[j * order + k] -= temp[1];

                arm_mult_q15(&matB[i * order + k], &temp[0], &temp[1], 1);
                matB[j * order + k] -= temp[1];
            }
        }
    }

    //Backward
    for (int i = order - 1; i >= 0; i--) {
        for (int j = i - 1; j >= 0; j--) {
            temp[0] = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                arm_mult_q15(&matA[i * order + k], &temp[0], &temp[1], 1);
                matA[j * order + k] -= temp[1];

                arm_mult_q15(&matB[i * order + k], &temp[0], &temp[1], 1);
                matB[j * order + k] -= temp[1];
            }
        }
    }

    arm_copy_q15(matB, result, order*order);
}
