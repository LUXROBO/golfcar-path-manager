#pragma once

#include <vector>
#include <qformat.h>

/**
 * @file model_matrix.h
 */

class ModelMatrix_D {
public:
    static constexpr int MAX_RAW = 7;
    static constexpr int MAX_COL = 7;
    static constexpr int MAX_SIZE = ModelMatrix_D::MAX_RAW * ModelMatrix_D::MAX_COL;
public:
    /**
     * @brief Create a new ModelMatrix_D instance.
     */
    ModelMatrix_D();

    /**
     * @brief Create a new ModelMatrix_D instance.
     */
    ModelMatrix_D(const ModelMatrix_D &other);

    /**
     * @brief Create a new ModelMatrix_D instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     */
    ModelMatrix_D(const unsigned int row, const unsigned int column);

    /**
     * @brief Create a new ModelMatrix_D instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] element matrix element.
     */
    ModelMatrix_D(const unsigned int row, const unsigned int column, const double *element);

    /**
     * @brief Create a new ModelMatrix_D instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] element matrix element.
     */
    ModelMatrix_D(const unsigned int row, const unsigned int column, const double **element);

    /**
     * @brief Create a new ModelMatrix_D instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] element matrix element.
     */
    ModelMatrix_D(const unsigned int row, const unsigned int column, const std::vector<double> element);

    /**
     * @brief Destructor
     */
    ~ModelMatrix_D();

public:
    /**
     * @brief get matrix row
     * @returns matrix row.
     */
    unsigned int row() const;

    /**
     * @brief get matrix column
     * @returns matrix column.
     */
    unsigned int column() const;

    /**
     * @brief get matrix element
     * @returns matrix element.
     */
    double* element() const;

    /**
     * @brief get matrix element
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns matrix element.
     */
    double get(const unsigned int row, const unsigned int column) const;

    /**
     * @brief set matrix element
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] value element value.
     */
    void set(const unsigned int row, const unsigned int column, const double value);

    /**
     * @brief calculate zero matrix
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns zero matrix.
     */
    static ModelMatrix_D zero(const unsigned int row, const unsigned int column);

    /**
     * @brief calculate one matrix
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns one matrix.
     */
    static ModelMatrix_D one(const unsigned int row, const unsigned int column);

    /**
     * @brief calculate identity matrix
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns identity matrix.
     */
    static ModelMatrix_D identity(const unsigned int row, const unsigned int column);

    /**
     * @brief calculate transpose
     * @returns result matrix.
     */
    ModelMatrix_D transpose();

    /**
     * @brief calculate determinant
     * @returns determinant.
     */
    double determinant();

    /**
     * @brief calculate inverse matrix
     * @returns result matrix.
     */
    ModelMatrix_D inverse();

    /**
     * @brief calculate inverse matrix with DLS
     * @param[in] sigma DLS sigma.
     * @returns result matrix.
     */
    ModelMatrix_D inverse(const double sigma);

    /**
     * @brief get vector length
     * @returns length.
     */
    double length() const;

    /**
     * @brief get normalized vector
     * @returns normalized vector.
     */
    ModelMatrix_D normalize() const;

    /**
     * @brief calculate vector dot product
     * @returns result value.
     */
    double dot(const ModelMatrix_D &rhs);

    /**
     * @brief calculate vector cross product
     * @returns result vector.
     */
    ModelMatrix_D cross(const ModelMatrix_D &rhs);

    /**
     * @brief convert vector to cross product matrix
     * @returns result matrix.
     */
    ModelMatrix_D cross();

    ModelMatrix_D &operator=(const ModelMatrix_D &other);
    ModelMatrix_D &operator=(const double* other);
    ModelMatrix_D operator+(const double &rhs);
    ModelMatrix_D operator+(const ModelMatrix_D &rhs);
    ModelMatrix_D operator-(const double &rhs);
    ModelMatrix_D operator-(const ModelMatrix_D &rhs);
    ModelMatrix_D operator*(const double &rhs);
    ModelMatrix_D operator*(const ModelMatrix_D &rhs);

    friend ModelMatrix_D operator+(const double &lhs, const ModelMatrix_D &rhs);
    friend ModelMatrix_D operator-(const double &lhs, const ModelMatrix_D &rhs);
    friend ModelMatrix_D operator*(const double &lhs, const ModelMatrix_D &rhs);

private:
    ModelMatrix_D pseudoInverse();
    ModelMatrix_D pseudoInverseR();
    ModelMatrix_D pseudoInverseL();
    double determinant(double* matrix, int order);
    ModelMatrix_D matrixInversion(double* matrix, int order);

private:
    unsigned int row_;
    unsigned int column_;
    double element_[ModelMatrix_D::MAX_RAW * ModelMatrix_D::MAX_COL];
};
