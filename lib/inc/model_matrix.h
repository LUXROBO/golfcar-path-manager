#ifndef MODEL_MATRIX_H
#define MODEL_MATRIX_H

#include <vector>
#include <qformat.h>

/**
 * @file model_matrix.h
 */

class ModelMatrix {
public:
    /**
     * @brief Create a new ModelMatrix instance.
     */
    ModelMatrix();

    /**
     * @brief Create a new ModelMatrix instance.
     */
    ModelMatrix(const ModelMatrix &other);

    /**
     * @brief Create a new ModelMatrix instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     */
    ModelMatrix(const unsigned int row, const unsigned int column);

    /**
     * @brief Create a new ModelMatrix instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] element matrix element.
     */
    ModelMatrix(const unsigned int row, const unsigned int column, const q_format *element);

    /**
     * @brief Create a new ModelMatrix instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] element matrix element.
     */
    ModelMatrix(const unsigned int row, const unsigned int column, const q_format **element);

    /**
     * @brief Create a new ModelMatrix instance.
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] element matrix element.
     */
    ModelMatrix(const unsigned int row, const unsigned int column, const std::vector<q_format> element);

    /**
     * @brief Destructor
     */
    ~ModelMatrix();

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
    std::vector<q_format> element() const;

    /**
     * @brief get matrix element
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns matrix element.
     */
    q_format get(const unsigned int row, const unsigned int column) const;

    /**
     * @brief set matrix element
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @param[in] value element value.
     */
    void set(const unsigned int row, const unsigned int column, const q_format value);

    /**
     * @brief calculate zero matrix
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns zero matrix.
     */
    static ModelMatrix zero(const unsigned int row, const unsigned int column);

    /**
     * @brief calculate one matrix
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns one matrix.
     */
    static ModelMatrix one(const unsigned int row, const unsigned int column);

    /**
     * @brief calculate identity matrix
     * @param[in] row matrix row.
     * @param[in] column matrix column.
     * @returns identity matrix.
     */
    static ModelMatrix identity(const unsigned int row, const unsigned int column);

    /**
     * @brief calculate transpose
     * @returns result matrix.
     */
    ModelMatrix transpose();

    /**
     * @brief calculate determinant
     * @returns determinant.
     */
    q_format determinant();

    /**
     * @brief calculate inverse matrix
     * @returns result matrix.
     */
    ModelMatrix inverse();

    /**
     * @brief calculate inverse matrix with DLS
     * @param[in] sigma DLS sigma.
     * @returns result matrix.
     */
    ModelMatrix inverse(const q_format sigma);

    /**
     * @brief get vector length
     * @returns length.
     */
    q_format length() const;

    /**
     * @brief get normalized vector
     * @returns normalized vector.
     */
    ModelMatrix normalize() const;

    /**
     * @brief calculate vector dot product
     * @returns result value.
     */
    q_format dot(const ModelMatrix &rhs);

    /**
     * @brief calculate vector cross product
     * @returns result vector.
     */
    ModelMatrix cross(const ModelMatrix &rhs);

    /**
     * @brief convert vector to cross product matrix
     * @returns result matrix.
     */
    ModelMatrix cross();

    ModelMatrix &operator=(const ModelMatrix &other);
    ModelMatrix operator+(const q_format &rhs);
    ModelMatrix operator+(const ModelMatrix &rhs);
    ModelMatrix operator-(const q_format &rhs);
    ModelMatrix operator-(const ModelMatrix &rhs);
    ModelMatrix operator*(const q_format &rhs);
    ModelMatrix operator*(const ModelMatrix &rhs);

    friend ModelMatrix operator+(const q_format &lhs, const ModelMatrix &rhs);
    friend ModelMatrix operator-(const q_format &lhs, const ModelMatrix &rhs);
    friend ModelMatrix operator*(const q_format &lhs, const ModelMatrix &rhs);

private:
    ModelMatrix pseudoInverse();
    ModelMatrix pseudoInverseR();
    ModelMatrix pseudoInverseL();
    q_format determinant(std::vector<q_format> matrix, int order);
    std::vector<q_format> matrixInversion(std::vector<q_format> matrix, int order);

private:
    unsigned int row_;
    unsigned int column_;
    std::vector<q_format> element_;
};

#endif // MODEL_MATRIX_H
