#pragma once

#include <array>
#include <cassert>
#include <vector>
#include <iostream>
#include <math.h>

namespace math
{
    struct matrix
    {
        std::vector<std::vector<double>> array;

        matrix() = delete;

        matrix(std::size_t rows, std::size_t cols);

        matrix(std::vector<std::vector<double>> array);

        std::size_t rows;
        std::size_t cols;
        // double array[ROWS][COLS];

        // std::array<std::array<double, COLS>, ROWS> array;
        // std::vector<std::vector<double>> array(rows, std::vector<double>(cols));

        matrix operator*(const matrix other);

        matrix operator+ (const matrix& other) const;

        matrix operator- (const matrix& other) const;

        void print();
        // General matrix inverse. Uses Gauss-Jordan metho
    };

    // template<std::size_t rows, std::size_t cols>
    matrix inv(const matrix& mat);

    matrix trans(const matrix& mat);
}
