#include "matrix.hpp"

namespace math
{
    matrix::matrix(std::size_t rows, std::size_t cols) : rows{ rows }, cols{ cols }
    {
        for (std::size_t i{}; i < rows; i++)
        {
            std::vector<double> temp_row;
            temp_row.resize(cols);
            array.push_back(temp_row);
        }
    };

    matrix::matrix(std::vector<std::vector<double>> array) : array{ array }, rows{ array.size() }, cols{ array.begin()->size() }
    {
    }

    matrix matrix::operator*(const matrix other)
    {
        assert(cols == other.rows);

        matrix product{ rows, other.cols };
        for (int current_col{}; current_col != other.cols; current_col++)
        {
            for (int current_row{}; current_row != rows; current_row++)
            {
                double dot_sum{};
                for (int first_col{}, second_row{}; first_col != cols; first_col++, second_row++)
                {
                    dot_sum += array[current_row][first_col] * other.array[second_row][current_col];
                }
                product.array[current_row][current_col] = dot_sum;
            }
        }

        return product;
    }

    matrix matrix::operator+ (const matrix& other) const
    {
        assert(rows == other.rows);
        assert(cols == other.cols);

        matrix sum{ other.array };
        for (std::size_t row{}; row < rows; row++)
        {
            for (std::size_t col{}; col < cols; col++)
                sum.array[row][col] += array[row][col];
        }
        return sum;
    }

    matrix matrix::operator- (const matrix& other) const
    {
        assert(rows == other.rows);
        assert(cols == other.cols);

        matrix diff{ array };
        for (std::size_t row{}; row < rows; row++)
        {
            for (std::size_t col{}; col < cols; col++)
                diff.array[row][col] -= other.array[row][col];
        }
        return diff;
    }

    void matrix::print()
    {
        for (auto& row : array)
        {
            for (auto& element : row)
                std::cout << element << "\t";
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    // template<std::size_t rows, std::size_t cols>
    matrix inv(const matrix& mat)
    {
        assert(mat.cols == mat.rows);

        const std::size_t& rows{mat.rows};
        const std::size_t& cols{mat.cols};
        // Expanded form [A|I]
		// std::array<double, rows * rows * 2> m{};
        std::vector<double> m;
        m.reserve(rows * rows * 2);

        for (std::size_t r{}; r < rows; ++r)
        {
            for (std::size_t c{}; c < cols; ++c)
            {
                // m[c + r * cols * 2] = mat.array[c + r * cols];
                m[c + r * cols * 2] = mat.array[r][c];
            }
            m[cols + r + r * cols * 2] = 1.0;
        }
        // Perform elimination [A|I] -> [I|A^-1]
        for (int i{}; i < rows; ++i)
        {
            // Divide i row by value at (i,i)
            const double scalar{1.0 / m[i + i * cols * 2]};
            assert(std::isfinite(scalar)); // fix: swap rows until we find non-zero at (i,i)
            for (int j{}; j < cols * 2; ++j)
            {
                m[j + i * cols * 2] *= scalar;
            }
            // For each row that is not i, subtract i row multiplied by (k, i)
            for (int k{}; k < rows; ++k)
            {
                if (k == i)
                {
                    continue;
                }
                const double v{m[i + k * cols * 2]};
                for (int j{}; j < cols * 2; ++j)
                {
                    m[j + k * cols * 2] -= v * m[j + i * cols * 2];
                }
            }
        }
        // Read out result
        matrix m_inv{rows, cols};
        for (int r{}; r < rows; ++r)
        {
            for (int c{}; c < cols; ++c)
            {
                m_inv.array[r][c] =  m[cols + c + r * cols * 2];
            }
        }
        return m_inv;
    }

    matrix trans(const matrix& mat)
    {
        matrix trans{mat.cols, mat.rows};

        for (std::size_t r{}; r < mat.cols; r++)
        {
            for (std::size_t c{}; c < mat.rows; c++)
            {
                trans.array[r][c] = mat.array[c][r];
            }
        }
        return trans;
    }
}