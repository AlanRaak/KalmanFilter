#pragma once

#include "../math/matrix.hpp"

#include <fstream>
#include <string.h>

namespace util
{
void state_to_file(std::ofstream& file, const math::matrix& state)
{
    file << state.array[0][0] << ", "
         << state.array[1][0] << ", "
         << state.array[2][0] << ", "
         << state.array[3][0] << ", "
         << state.array[4][0] << "\n";
}
}