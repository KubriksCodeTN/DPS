// WARNING: non-standard pragma
#pragma once

#include <iostream>
#include <vector>
#include <tuple>
#include <iomanip>
#include <fstream>
#include <cassert>
#include <chrono>

#include "dubins.hpp"

namespace multi_dubins{

    using point_t = std::tuple<double, double>;
    using path_t = std::vector<dubins::d_curve>;
    using std::tuple;
    using std::get;
    using std::vector;

    path_t dp_dubins(const vector<point_t>&, double, double, double, uint32_t, uint32_t);

};