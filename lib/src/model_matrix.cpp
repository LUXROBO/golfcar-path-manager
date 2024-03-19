#include "model_matrix.h"
#include "string.h"
#include <cmath>


// ModelMatrix
ModelMatrix::ModelMatrix()
    : row_(4), column_(4) {
    memset((void*)element_, 0, sizeof(q_format) * ModelMatrix::MAX_SIZE);
}

ModelMatrix::ModelMatrix(const ModelMatrix &other)
    : row_(other.row()), column_(other.column()) {
    memcpy((void*)element_, (void*)other.element(), sizeof(q_format) * ModelMatrix::MAX_SIZE);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column)
    : row_(row), column_(column) {
    memset((void*)element_, 0, sizeof(q_format) * ModelMatrix::MAX_SIZE);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const q_format *element)
    : row_(row), column_(column) {
    memset((void*)element_, 0, sizeof(q_format) * ModelMatrix::MAX_SIZE);
    memcpy((void*)element_, (void*)element, sizeof(q_format) * row * column);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const q_format **element)
    : row_(row), column_(column) {
    memset((void*)element_, 0, sizeof(q_format) * ModelMatrix::MAX_SIZE);

    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r][c];
        }
    }
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const std::vector<q_format> element)
    : row_(row), column_(column){
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r * column + c];
        }
    }
}

ModelMatrix::~ModelMatrix() {
    memset((void*)element_, 0, sizeof(q_format) * ModelMatrix::MAX_SIZE);
}

unsigned int ModelMatrix::row() const {
    return row_;
}

unsigned int ModelMatrix::column() const {
    return column_;
}

q_format* ModelMatrix::element() const {
    return (q_format*)element_;
}

q_format ModelMatrix::get(const unsigned int row, const unsigned int column) const {
    if (row > row_) {
        return q_format();
    } else if (column > column_) {
        return q_format();
    } else {
        return element_[row * column_ + column];
    }
}

void ModelMatrix::set(const unsigned int row, const unsigned int column, const q_format value) {
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
    q_format mat[ModelMatrix::MAX_SIZE];
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            // mat[r * column + c] = 1.0;
            mat[r * column + c] = q_format(q_format::default_one, q_format::init_q_format_flag);
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::identity(const unsigned int row, const unsigned int column) {
    q_format mat[ModelMatrix::MAX_SIZE];
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            if (r == c) {
                mat[r * column + c] = q_format(q_format::default_one, q_format::init_q_format_flag);
            } else {
                mat[r * column + c] = q_format();
            }
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::transpose() {
    q_format ele[ModelMatrix::MAX_SIZE];
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            ele[c * row_ + r] = element_[r * column_ + c];
        }
    }
    return ModelMatrix(column_, row_, ele);
}

q_format ModelMatrix::determinant() {
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
        return matrixInversion(element_, row_);
    } else {
        // rectangular matrix
        return pseudoInverse();
    }
}

ModelMatrix ModelMatrix::inverse(const q_format sigma) {
    if (row_ <= column_) {
        // m by n matrix (n >= m)
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        ModelMatrix temp2 = (*this) * (this->transpose());
        ModelMatrix temp3 = temp2 + temp;
        return this->transpose() * temp3.inverse();
    } else {
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return (this->transpose() * (*this) + temp).inverse() * this->transpose();
    }
}

q_format ModelMatrix::length() const {
    q_format l = 0.0;
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            l = l + (element_[r * column_ + c] * element_[r * column_ + c]);
        }
    }
    return std::sqrt(l.to_double());
}

ModelMatrix ModelMatrix::normalize() const {
    q_format l = length();
    if (l == 0.0) {
        return ModelMatrix::identity(row_, column_);
    } else {
        q_format ele[ModelMatrix::MAX_SIZE];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                ele[r * column_ + c] = element_[r * column_ + c] / l;
            }
        }
        return ModelMatrix(row_, column_, ele);
    }
}

q_format ModelMatrix::dot(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        q_format dot = 0.0;
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                dot = dot + element_[r * column_ + c] * rhs.element()[r * column_ + c];
            }
        }
        return dot;
    } else {
        return 0.0;
    }
}

ModelMatrix ModelMatrix::cross(const ModelMatrix &rhs) {
    if (row_ == 3 && column_ == 1 && rhs.row() == 3 && rhs.column() == 1) {
        q_format ele[ModelMatrix::MAX_SIZE];
        ele[0] = element_[1] * rhs.element()[2] - element_[2] * rhs.element()[1];
        ele[1] = element_[2] * rhs.element()[0] - element_[0] * rhs.element()[2];
        ele[2] = element_[0] * rhs.element()[1] - element_[1] * rhs.element()[0];
        return ModelMatrix(row_, column_, ele);
    } else {
        return ModelMatrix::zero(3, 1);
    }
}

ModelMatrix ModelMatrix::cross() {
    if (row_ == 3 && column_ == 1) {
        q_format ele[ModelMatrix::MAX_SIZE];
        ele[0 * 3 + 0] = 0.0;
        ele[0 * 3 + 1] = element_[2] * -1;
        ele[0 * 3 + 2] = element_[1];
        ele[1 * 3 + 0] = element_[2];
        ele[1 * 3 + 1] = 0.0;
        ele[1 * 3 + 2] = element_[0] * -1;
        ele[2 * 3 + 0] = element_[1] * -1;
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
    memcpy((void*)this->element_, (void*)other.element(), sizeof(q_format) * ModelMatrix::MAX_SIZE);
    return *this;
}

ModelMatrix ModelMatrix::operator+(const q_format &rhs) {
    ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
	return (*this) + right;
}

ModelMatrix ModelMatrix::operator+(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        q_format temp[ModelMatrix::MAX_SIZE];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp[r * column_ + c] = element_[r * column_ + c] + rhs.element()[r * column_ + c];
            }
        }
        return ModelMatrix(row_, column_, temp);
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator-(const q_format &rhs) {
    ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
	return (*this) - right;
}

ModelMatrix ModelMatrix::operator-(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        q_format temp[ModelMatrix::MAX_SIZE];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp[r * column_ + c] = element_[r * column_ + c] - rhs.element()[r * column_ + c];
            }
        }
        return ModelMatrix(row_, column_, temp);
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator*(const q_format &rhs) {
    q_format temp[ModelMatrix::MAX_SIZE] = {0, };
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            temp[r * column_ + c] = element_[r * column_ + c] * rhs;
        }
    }
    return ModelMatrix(row_, column_, temp);
}

ModelMatrix ModelMatrix::operator*(const ModelMatrix &rhs) {
    if (column_ == rhs.row()) {
		q_format temp[ModelMatrix::MAX_SIZE] = {0, };
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < rhs.column(); c++) {
                temp[r * rhs.column() + c] = 0;
                for (unsigned int k = 0; k < column_; k++) {
                    temp[r * rhs.column() + c] += element_[r * column_ + k] * rhs.element()[k * rhs.column() + c];
                }
            }
        }
        return ModelMatrix(row_, rhs.column(), temp);
    } else {
		return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix operator+(const q_format &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row(), rhs.column()) * lhs;
    return left + rhs;
}

ModelMatrix operator-(const q_format &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row(), rhs.column()) * lhs;
    return left - rhs;
}

ModelMatrix operator*(const q_format &lhs, const ModelMatrix &rhs) {
    q_format temp[ModelMatrix::MAX_SIZE];
    for (unsigned int r = 0; r < rhs.row(); r++) {
        for (unsigned int c = 0; c < rhs.column(); c++) {
            temp[r * rhs.column() + c] = rhs.element()[r * rhs.column() + c] * lhs;
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

q_format ModelMatrix::determinant(q_format* matrix, int order) {
    // the determinant value
    q_format det = 1.0;

    // stop the recursion when matrix is a single element
    if (order == 1) {
        det = matrix[0];
    } else if (order == 2) {
        det = matrix[0 * 2 + 0] * matrix[1 * 2 + 1] - matrix[0 * 2 + 1] * matrix[1 * 2 + 0];
    } else if (order == 3) {
        det = matrix[0 * 3 + 0] * matrix[1 * 3 + 1] * matrix[2 * 3 + 2] + matrix[0 * 3 + 1] * matrix[1 * 3 + 2] * matrix[2 * 3 + 0] + matrix[0 * 3 + 2] * matrix[1 * 3 + 0] * matrix[2 * 3 + 1] - matrix[0 * 3 + 0] * matrix[1 * 3 + 2] * matrix[2 * 3 + 1] - matrix[0 * 3 + 1] * matrix[1 * 3 + 0] * matrix[2 * 3 + 2] - matrix[0 * 3 + 2] * matrix[1 * 3 + 1] * matrix[2 * 3 + 0];
    } else {
        // generation of temporary matrix
        q_format temp_matrix[ModelMatrix::MAX_SIZE];
        memcpy((void*)temp_matrix, (void*)matrix, sizeof(q_format) * this->row_ * this->column_);
        // std::vector<q_format> temp_matrix = matrix;

        // gaussian elimination
        for (int i = 0; i < order; i++) {
            // find max low
            q_format temp = 0.000;
            int max_row = i;
            for (int j = i; j < order; j++) {
                if (temp_matrix[j * order + i].abs() > temp) {
                    temp = temp_matrix[j * order + i].abs();
                    max_row = j;
                }
            }
            if (temp_matrix[max_row * order + i].abs() > 0.0001) {
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
                    temp = temp_matrix[j * order + i] / temp_matrix[i * order + i];
                    for (int k = i; k < order; k++) {
                        temp_matrix[j * order + k] = temp_matrix[j * order + k] - temp_matrix[i * order + k] * temp;
                    }
                }
            }
        }

        for (int i = 0; i < order; i++) {
            det = det * temp_matrix[i * order + i];
        }
    }

    return det;
}
static q_format matddA[ModelMatrix::MAX_SIZE];
ModelMatrix ModelMatrix::matrixInversion(q_format* matrix, int order) {
    // std::vector<q_format> matA = matrix;
    q_format matA[ModelMatrix::MAX_SIZE];
    memcpy((void*)matA, (void*)matrix, sizeof(q_format) * this->row_ * this->column_);
    memset((void*)matddA, 0, sizeof(q_format) * ModelMatrix::MAX_SIZE);
    memcpy((void*)matddA, (void*)matA, sizeof(q_format) * this->row_ * this->column_);
    q_format matB[ModelMatrix::MAX_SIZE];
    memcpy((void*)matB, (void*)ModelMatrix::identity(order, order).element(), sizeof(q_format) * order * order);

    // Gauss-Jordan
    // Forward
    for (int i = 0; i < order; i++) {
        // max row
        q_format temp = 0;
        int max_row = i;
        for (int j = i; j < order; j++) {
            if (matA[j * order + i].abs() > temp) {
                temp = matA[j * order + i].abs();
                max_row = j;
            }
        }
        // change row
        q_format temp2 = matA[max_row * order + i];
        for (int j = 0; j < order; j++) {
            q_format temp3 = matA[max_row * order + j];
            matA[max_row * order + j] = matA[i * order + j];
            matA[i * order + j] = temp3 / temp2;

            temp3 = matB[max_row * order + j];
            matB[max_row * order + j] = matB[i * order + j];
            matB[i * order + j] = temp3 / temp2;
        }
        for (int j = i + 1; j < order; j++) {
            q_format temp3 = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                matA[j * order + k] -= matA[i * order + k] * temp3;
                matB[j * order + k] -= matB[i * order + k] * temp3;
            }
        }
    }

    //Backward
    for (int i = order - 1; i >= 0; i--) {
        for (int j = i - 1; j >= 0; j--) {
            q_format temp = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                matA[j * order + k] -= matA[i * order + k] * temp;
                matB[j * order + k] -= matB[i * order + k] * temp;
            }
        }
    }

    ModelMatrix td(order, order, matB);

    return td;
}
