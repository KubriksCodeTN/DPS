#include "planner.hpp"

#include <random>
#include <functional>
#include <chrono>
#include <cmath>
#include <cassert>


using path_d = std::vector<VisiLibity::Point>;

#define __DEBUG
#ifdef __DEBUG

path_d sample_arc(dubins::d_arc, uint_fast32_t);


/**
 * @brief creates a dump to visualize the clipper enviroment on desmos
 */
void desmos_dump(auto clipper_env){
    for (const auto& s : clipper_env){
        std::cout << "\\operatorname{polygon}(";
        for (const auto t : s)
            std::cout << "(" << t.x() << ", " << t.y() << "), ";
        std::cout << "\b\b)\n";
    }
    std::cout << "---------------------------------------------\n";
}

/**
 * @brief creates a dump to visualize the shortest path on desmos
 */
void path_dump(auto path){
    if(path.size() >= 500)
        return;
    std::cout << "Polyline:\n";
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << std::fixed << "\\operatorname{polygon}((" << path[i].x() << ", " << path[i].y() << "), (";
        std::cout << std::fixed << path[i + 1].x() << ", " << path[i + 1].y() << "))\n";
    }
    std::cout << "---------------------------------------------\n";
}

/**
 * @brief creates a dump to visualize a dubins arc on desmos
 */
void arc_dump(auto path){
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << std::fixed << "\\operatorname{polygon}((" << path[i].x() << ", " << path[i].y() << "), (";
        std::cout << std::fixed << path[i + 1].x() << ", " << path[i + 1].y() << "))\n";
    }
    std::cout << "---------------------------------------------\n";
}


/**
 * @brief creates a dump to visualize the dubins shortest path on desmos
 */
void dubins_dump(const auto& dpath){
    if(dpath.size() >= 500)
        return;
    std::cout << "Dubins path:\n";
    for (const auto& c : dpath){
        arc_dump(sample_arc(c.a1, 4));
        arc_dump(sample_arc(c.a2, 4));
        arc_dump(sample_arc(c.a3, 4));
    }
    std::cout << "---------------------------------------------\n";
}

#define LOG(...) fprintf(stderr, __VA_ARGS__)

#else

#define desmos_dump(x)
#define path_dump(x)
#define dubins_dump(x)
#define LOG(x...)

#endif

/**
 * @brief numerically stable implementation of the sinc function
 * 
 * @note yes, this is duplicated code
 */
inline constexpr double sinc(double x){
    if (fabs(x) > 0.002)
        return sin(x) / x;
    return 1 - x * x / 6 * (1 - x * x / 20);
}

/**
 * @brief sample a point from a dubins arc
 */
VisiLibity::Point circline(double x, double y, double th, double s, double k){
    double sin, cos;
    sincos(th + k * s / 2., &sin, &cos);
    return {
        x + s * sinc(k * s / 2.) * cos,
        y + s * sinc(k * s / 2.) * sin,
    };
}

/**
 * @brief sample a dubins arc
 * 
 * @param arc the dubins arc to sample
 * @param n_samples fast number of samples needed. 
 *      "How can it be this fast!?" 
 *      "It just is."
 */
path_d sample_arc(dubins::d_arc arc, uint_fast32_t n_samples = 30){
    path_d v{ n_samples + 1 };

    // &p - &v[0] is a nice hack to get the element idx without race conditions
    std::for_each(v.begin(), v.end(), [&](auto& p){
        double s = arc.L / n_samples * (&p - &v[0]);
        p = circline(arc.x0, arc.y0, arc.th0, s, arc.k);
    });

    return v;
}

/**
 * @brief create the dubins path following the proposed strategies 
 */
multi_dubins::path_t Planner::dubins_path(const VisiLibity::Polyline& path, double th0, double thf, double r){

    /*
    double th;
    multi_dubins::path_t start;
    multi_dubins::path_t end;
    switch (path.size()){
    case 2:
        return sample_curve(path[0], th0, path[1], thf, 1. / min_r);
    case 3:
        // note th as intermediate angle might not always be the best but it looks like 
        // a decent choice for a O(1) time algorithm
        th = atan2(path[2].y() - path[1].y(), path[2].x() - path[1].x());
        start = sample_curve(path[0], th0, path[1], th, 1. / min_r);
        end = sample_curve(path[1], th, path[2], thf, 1. / min_r);
        std::ranges::move(end, std::back_insert_iterator(start));
        return start;
    case 4:
        th = atan2(path[2].y() - path[1].y(), path[2].x() - path[1].x());
        dubins::d_curve safe_curve;
        dubins::d_shortest(path[1].x(), path[1].y(), th, path[2].x(), path[2].y(), th, 1. / min_r, safe_curve);
        start = sample_curve(path[0], th0, path[1], th, 1. / min_r);
        start.push_back(safe_curve); // always a straight line, in line with our intermediate curves strategy
        end = sample_curve(path[2], th, path[3], thf, 1. / min_r);
        std::ranges::move(end, std::back_insert_iterator(start));
        return start;
    }
    */

    // general case
    multi_dubins::path_t sol{ path.size() - 1 };
    VisiLibity::Point new_a;

    dubins_wrapper(path, sol, new_a, r);
    
    /*
    start = sample_curve(path[0], th0, path[1], sol[1].a1.th0, 1. / min_r);
    end = sample_curve(new_a, sol[sol.size() - 2].a3.thf, path[path.size() - 1], thf, 1. / min_r);
    sol.front() = start[0];
    if (start.size() > 1)
        sol.insert(sol.begin() + 1, start[1]);
    std::ranges::move(end, std::back_insert_iterator(sol));
    */
    return sol;
}

void Planner::test(const VisiLibity::Polyline& shortest_path){


    double th0 = atan2(shortest_path[1].y() - shortest_path[0].y(), shortest_path[1].x() - shortest_path[0].x());
    double thf = atan2(shortest_path[shortest_path.size() - 1].y() - shortest_path[shortest_path.size() - 2].y(), 
        shortest_path[shortest_path.size() - 1].x() - shortest_path[shortest_path.size() - 2].x());
    
    // get the path made out of dubin curves
    multi_dubins::path_t p0 = dubins_path(shortest_path, th0, thf, inv_k);
    
    path_dump(shortest_path);
    dubins_dump(p0);
    double len = 0;
    for(int i = 0; i < p0.size(); ++i)
        len += p0[i].L;

    std::cout << "Total length: " << len << "\n";
}
