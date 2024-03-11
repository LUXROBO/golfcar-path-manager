#include "model_matrix_double.h"
#include "string.h"
#include <cmath>


// ModelMatrix_D
ModelMatrix_D::ModelMatrix_D()
    : row_(4), column_(4) {
    memset((void*)element_, 0, sizeof(double) * ModelMatrix_D::MAX_SIZE);
}

ModelMatrix_D::ModelMatrix_D(const ModelMatrix_D &other)
    : row_(other.row()), column_(other.column()) {
    memcpy((void*)element_, (void*)other.element(), sizeof(double) * ModelMatrix_D::MAX_SIZE);
}

ModelMatrix_D::ModelMatrix_D(const unsigned int row, const unsigned int column)
    : row_(row), column_(column) {
    memset((void*)element_, 0, sizeof(double) * ModelMatrix_D::MAX_SIZE);
}

ModelMatrix_D::ModelMatrix_D(const unsigned int row, const unsigned int column, const double *element)
    : row_(row), column_(column) {
    memset((void*)element_, 0, sizeof(double) * ModelMatrix_D::MAX_SIZE);
    memcpy((void*)element_, (void*)element, sizeof(double) * row * column);
}

ModelMatrix_D::ModelMatrix_D(const unsigned int row, const unsigned int column, const double **element)
    : row_(row), column_(column) {
    memset((void*)element_, 0, sizeof(double) * ModelMatrix_D::MAX_SIZE);

    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r][c];
        }
    }
}

ModelMatrix_D::ModelMatrix_D(const unsigned int row, const unsigned int column, const std::vector<double> element)
    : row_(row), column_(column){
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r * column + c];
        }
    }
}

ModelMatrix_D::~ModelMatrix_D() {
    memset((void*)element_, 0, sizeof(double) * ModelMatrix_D::MAX_SIZE);
}

unsigned int ModelMatrix_D::row() const {
    return row_;
}

unsigned int ModelMatrix_D::column() const {
    return column_;
}

double* ModelMatrix_D::element() const {
    return (double*)element_;
}

double ModelMatrix_D::get(const unsigned int row, const unsigned int column) const {
    if (row > row_) {
        return 0;
    } else if (column > column_) {
        return 0;
    } else {
        return element_[row * column_ + column];
    }
}

void ModelMatrix_D::set(const unsigned int row, const unsigned int column, const double value) {
    if (row > row_) {
        return;
    } else if (column > column_) {
        return;
    } else {
        element_[row * column_ + column] = value;
    }
}

ModelMatrix_D ModelMatrix_D::zero(const unsigned int row, const unsigned int column) {
    return ModelMatrix_D(row, column);
}

ModelMatrix_D ModelMatrix_D::one(const unsigned int row, const unsigned int column) {
    double mat[ModelMatrix_D::MAX_SIZE];
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            // mat[r * column + c] = 1.0;
            mat[r * column + c] = 1;
        }
    }
    return ModelMatrix_D(row, column, mat);
}

ModelMatrix_D ModelMatrix_D::identity(const unsigned int row, const unsigned int column) {
    double mat[ModelMatrix_D::MAX_SIZE];
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            if (r == c) {
                mat[r * column + c] = 1;
            } else {
                mat[r * column + c] = 0;
            }
        }
    }
    return ModelMatrix_D(row, column, mat);
}

ModelMatrix_D ModelMatrix_D::transpose() {
    double ele[ModelMatrix_D::MAX_SIZE];
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            ele[c * row_ + r] = element_[r * column_ + c];
        }
    }
    return ModelMatrix_D(column_, row_, ele);
}

double ModelMatrix_D::determinant() {
    if (row_ == column_) {
        return determinant(element_, row_);
    } else if (row_ > column_) {
        return determinant((this->transpose() * (*this)).element(), column_);
    } else {
        return determinant(((*this) * this->transpose()).element(), row_);
    }
}

ModelMatrix_D ModelMatrix_D::inverse() {
    if (row_ == column_) {
        // square matrix
        return matrixInversion(element_, row_);
    } else {
        // rectangular matrix
        return pseudoInverse();
    }
}

ModelMatrix_D ModelMatrix_D::inverse(const double sigma) {
    if (row_ <= column_) {
        // m by n matrix (n >= m)
        // generate sigma digonal matrix
        ModelMatrix_D temp = ModelMatrix_D::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        ModelMatrix_D temp2 = (*this) * (this->transpose());
        ModelMatrix_D temp3 = temp2 + temp;
        return this->transpose() * temp3.inverse();
    } else {
        // generate sigma digonal matrix
        ModelMatrix_D temp = ModelMatrix_D::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return (this->transpose() * (*this) + temp).inverse() * this->transpose();
    }
}

double ModelMatrix_D::length() const {
    double l = 0.0;
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            l = l + (element_[r * column_ + c] * element_[r * column_ + c]);
        }
    }
    return std::sqrt(l);
}

ModelMatrix_D ModelMatrix_D::normalize() const {
    double l = length();
    if (l == 0.0) {
        return ModelMatrix_D::identity(row_, column_);
    } else {
        double ele[ModelMatrix_D::MAX_SIZE];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                ele[r * column_ + c] = element_[r * column_ + c] / l;
            }
        }
        return ModelMatrix_D(row_, column_, ele);
    }
}

double ModelMatrix_D::dot(const ModelMatrix_D &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        double dot = 0.0;
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

ModelMatrix_D ModelMatrix_D::cross(const ModelMatrix_D &rhs) {
    if (row_ == 3 && column_ == 1 && rhs.row() == 3 && rhs.column() == 1) {
        double ele[ModelMatrix_D::MAX_SIZE];
        ele[0] = element_[1] * rhs.element()[2] - element_[2] * rhs.element()[1];
        ele[1] = element_[2] * rhs.element()[0] - element_[0] * rhs.element()[2];
        ele[2] = element_[0] * rhs.element()[1] - element_[1] * rhs.element()[0];
        return ModelMatrix_D(row_, column_, ele);
    } else {
        return ModelMatrix_D::zero(3, 1);
    }
}

ModelMatrix_D ModelMatrix_D::cross() {
    if (row_ == 3 && column_ == 1) {
        double ele[ModelMatrix_D::MAX_SIZE];
        ele[0 * 3 + 0] = 0.0;
        ele[0 * 3 + 1] = element_[2] * -1;
        ele[0 * 3 + 2] = element_[1];
        ele[1 * 3 + 0] = element_[2];
        ele[1 * 3 + 1] = 0.0;
        ele[1 * 3 + 2] = element_[0] * -1;
        ele[2 * 3 + 0] = element_[1] * -1;
        ele[2 * 3 + 1] = element_[0];
        ele[2 * 3 + 2] = 0.0;
        return ModelMatrix_D(3, 3, ele);
    } else {
        return ModelMatrix_D::zero(3, 3);
    }
}

ModelMatrix_D &ModelMatrix_D::operator=(const ModelMatrix_D &other) {
    this->row_ = other.row_;
    this->column_ = other.column();
    memcpy((void*)this->element_, (void*)other.element(), sizeof(double) * ModelMatrix_D::MAX_SIZE);
    return *this;
}

ModelMatrix_D &ModelMatrix_D::operator=(const double* other) {
    for (int i = 0; i < this->row_; i++) {
        for (int j = 0; i < this->column_; j++) {
            if(other == nullptr) {
                return *this;
            }
            this->set(i, j, *other++);
        }
    }
    return *this;
}

ModelMatrix_D ModelMatrix_D::operator+(const double &rhs) {
    ModelMatrix_D right = ModelMatrix_D::one(row_, column_) * rhs;
	return (*this) + right;
}

ModelMatrix_D ModelMatrix_D::operator+(const ModelMatrix_D &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        double temp[ModelMatrix_D::MAX_SIZE];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp[r * column_ + c] = element_[r * column_ + c] + rhs.element()[r * column_ + c];
            }
        }
        return ModelMatrix_D(row_, column_, temp);
    } else {
        return ModelMatrix_D::zero(row_, column_);
    }
}

ModelMatrix_D ModelMatrix_D::operator-(const double &rhs) {
    ModelMatrix_D right = ModelMatrix_D::one(row_, column_) * rhs;
	return (*this) - right;
}

ModelMatrix_D ModelMatrix_D::operator-(const ModelMatrix_D &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        double temp[ModelMatrix_D::MAX_SIZE];
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp[r * column_ + c] = element_[r * column_ + c] - rhs.element()[r * column_ + c];
            }
        }
        return ModelMatrix_D(row_, column_, temp);
    } else {
        return ModelMatrix_D::zero(row_, column_);
    }
}

ModelMatrix_D ModelMatrix_D::operator*(const double &rhs) {
    double temp[ModelMatrix_D::MAX_SIZE] = {0, };
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            temp[r * column_ + c] = element_[r * column_ + c] * rhs;
        }
    }
    return ModelMatrix_D(row_, column_, temp);
}

ModelMatrix_D ModelMatrix_D::operator*(const ModelMatrix_D &rhs) {
    if (column_ == rhs.row()) {
		double temp[ModelMatrix_D::MAX_SIZE] = {0, };
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < rhs.column(); c++) {
                temp[r * rhs.column() + c] = 0;
                for (unsigned int k = 0; k < column_; k++) {
                    temp[r * rhs.column() + c] += element_[r * column_ + k] * rhs.element()[k * rhs.column() + c];
                }
            }
        }
        return ModelMatrix_D(row_, rhs.column(), temp);
    } else {
		return ModelMatrix_D::zero(row_, column_);
    }
}

ModelMatrix_D operator+(const double &lhs, const ModelMatrix_D &rhs) {
    ModelMatrix_D left = ModelMatrix_D::one(rhs.row(), rhs.column()) * lhs;
    return left + rhs;
}

ModelMatrix_D operator-(const double &lhs, const ModelMatrix_D &rhs) {
    ModelMatrix_D left = ModelMatrix_D::one(rhs.row(), rhs.column()) * lhs;
    return left - rhs;
}

ModelMatrix_D operator*(const double &lhs, const ModelMatrix_D &rhs) {
    double temp[ModelMatrix_D::MAX_SIZE];
    for (unsigned int r = 0; r < rhs.row(); r++) {
        for (unsigned int c = 0; c < rhs.column(); c++) {
            temp[r * rhs.column() + c] = rhs.element()[r * rhs.column() + c] * lhs;
        }
    }
    return ModelMatrix_D(rhs.row(), rhs.column(), temp);
}

ModelMatrix_D ModelMatrix_D::pseudoInverse() {
    if (row_ == column_) {
        return inverse();
    } else if (row_ > column_) {
        return pseudoInverseL();
    } else {
        return pseudoInverseR();
    }
}

ModelMatrix_D ModelMatrix_D::pseudoInverseR() {
    return this->transpose() * ((*this) * (this->transpose())).inverse();
}

ModelMatrix_D ModelMatrix_D::pseudoInverseL() {
    return ((this->transpose()) * (*this)).inverse() * this->transpose();
}

double ModelMatrix_D::determinant(double* matrix, int order) {
    // the determinant value
    double det = 1.0;

    // stop the recursion when matrix is a single element
    if (order == 1) {
        det = matrix[0];
    } else if (order == 2) {
        det = matrix[0 * 2 + 0] * matrix[1 * 2 + 1] - matrix[0 * 2 + 1] * matrix[1 * 2 + 0];
    } else if (order == 3) {
        det = matrix[0 * 3 + 0] * matrix[1 * 3 + 1] * matrix[2 * 3 + 2] + matrix[0 * 3 + 1] * matrix[1 * 3 + 2] * matrix[2 * 3 + 0] + matrix[0 * 3 + 2] * matrix[1 * 3 + 0] * matrix[2 * 3 + 1] - matrix[0 * 3 + 0] * matrix[1 * 3 + 2] * matrix[2 * 3 + 1] - matrix[0 * 3 + 1] * matrix[1 * 3 + 0] * matrix[2 * 3 + 2] - matrix[0 * 3 + 2] * matrix[1 * 3 + 1] * matrix[2 * 3 + 0];
    } else {
        // generation of temporary matrix
        double temp_matrix[ModelMatrix_D::MAX_SIZE];
        memcpy((void*)temp_matrix, (void*)matrix, sizeof(double) * this->row_ * this->column_);
        // std::vector<double> temp_matrix = matrix;

        // gaussian elimination
        for (int i = 0; i < order; i++) {
            // find max low
            double temp = 0.000;
            int max_row = i;
            for (int j = i; j < order; j++) {
                if (abs(temp_matrix[j * order + i]) > temp) {
                    temp = abs(temp_matrix[j * order + i]);
                    max_row = j;
                }
            }
            if (abs(temp_matrix[max_row * order + i]) > 0.0001) {
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
double matddA[ModelMatrix_D::MAX_SIZE];
ModelMatrix_D ModelMatrix_D::matrixInversion(double* matrix, int order) {
    // std::vector<double> matA = matrix;
    double matA[ModelMatrix_D::MAX_SIZE];
    memcpy((void*)matA, (void*)matrix, sizeof(double) * this->row_ * this->column_);
    memset((void*)matddA, 0, sizeof(double) * ModelMatrix_D::MAX_SIZE);
    memcpy((void*)matddA, (void*)matA, sizeof(double) * this->row_ * this->column_);
    double matB[ModelMatrix_D::MAX_SIZE];
    memcpy((void*)matB, (void*)ModelMatrix_D::identity(order, order).element(), sizeof(double) * order * order);

    // Gauss-Jordan
    // Forward
    for (int i = 0; i < order; i++) {
        // max row
        double temp = 0;
        int max_row = i;
        for (int j = i; j < order; j++) {
            if (abs(matA[j * order + i]) > temp) {
                temp = abs(matA[j * order + i]);
                max_row = j;
            }
        }
        // change row
        double temp2 = matA[max_row * order + i];
        for (int j = 0; j < order; j++) {
            double temp3 = matA[max_row * order + j];
            matA[max_row * order + j] = matA[i * order + j];
            matA[i * order + j] = temp3 / temp2;

            temp3 = matB[max_row * order + j];
            matB[max_row * order + j] = matB[i * order + j];
            matB[i * order + j] = temp3 / temp2;
        }
        for (int j = i + 1; j < order; j++) {
            double temp3 = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                matA[j * order + k] -= matA[i * order + k] * temp3;
                matB[j * order + k] -= matB[i * order + k] * temp3;
            }
        }
    }

    //Backward
    for (int i = order - 1; i >= 0; i--) {
        for (int j = i - 1; j >= 0; j--) {
            double temp = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                matA[j * order + k] -= matA[i * order + k] * temp;
                matB[j * order + k] -= matB[i * order + k] * temp;
            }
        }
    }

    ModelMatrix_D td(order, order, matB);

    return td;
}
