// WARNING: non-standard pragma
#pragma once

#include <vector>
#include <ranges>
#include "multi_dubins/multi_dubins.hpp"

// since the visilibity library has some warnings that we can't really handle
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "visilibity/visilibity.hpp"
#pragma GCC diagnostic pop

class Planner{
    protected:
        dubins::d_curve get_safe_curve(VisiLibity::Point, VisiLibity::Point, VisiLibity::Point, VisiLibity::Point&, double);
        multi_dubins::path_t sample_curve(VisiLibity::Point, double, VisiLibity::Point, double, double);
        // void dubins_seq(const VisiLibity::Polyline&, multi_dubins::path_t&, VisiLibity::Point&, double);
        // void dubins_cuda(const VisiLibity::Polyline&, multi_dubins::path_t&, VisiLibity::Point&, double);
        double dubins_wrapper(const VisiLibity::Polyline&, multi_dubins::path_t&, VisiLibity::Point&, double);

    public:
        // robot constraints
        static inline constexpr double min_r = .5;
        static inline constexpr double inv_k = .5;
        static inline constexpr double hrobot_sz = .4;
        // static inline constexpr double velocity = x; // not needed?

        Planner() = default;
        multi_dubins::path_t dubins_path(const VisiLibity::Polyline&, double, double, double, double&);
        void test(const VisiLibity::Polyline&, double&, double&);
};
