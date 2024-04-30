#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <cmath>
#include <limits>

namespace dubins{
    
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

    struct d_curve{
        d_arc a1;
        d_arc a2;
        d_arc a3;
        double L;
    };

    int32_t d_shortest(double, double, double, double, double, double, double, d_curve&);
    std::vector<d_curve> d_paths(double, double, double, double, double, double, double);
};
