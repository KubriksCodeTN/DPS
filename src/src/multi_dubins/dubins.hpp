/**
 * @file dubins.hpp
 * @brief header of the Dubins curve implementation
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <cmath>
#include <limits>

/**
 * @brief Dubins namespace
 */
namespace dubins{
    /** @brief arc of a Dubins curve */
    struct d_arc{
        double x0;
        double y0;
        double th0;
        double k;
        double L;
        double xf;
        double yf;
        double thf;
    };

    /** @brief represents a Dubins curve */
    struct d_curve{
        d_arc a1;
        d_arc a2;
        d_arc a3;
        double L;
    };

    int32_t d_shortest(double, double, double, double, double, double, double, d_curve&);
    std::vector<d_curve> d_paths(double, double, double, double, double, double, double);
};
