/**
 * @file main.cpp
 * @brief file for the roadmap test
 */
#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <execution>
#include <random>
#include <chrono>
#include "clipper.h"
#include "multi_dubins.hpp"
#include "dp.hh"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "visilibity.hpp"
#pragma GCC diagnostic pop

using point_t =  std::tuple<double, double>;
using square_t = std::tuple<point_t, point_t, point_t, point_t>;

static constexpr double h = .4;
static std::mt19937 mt{std::random_device{}()};

#define g(x) std::get<x>

using path_d = std::vector<VisiLibity::Point>;

/*
inline constexpr double sinc(double x){
    if (fabs(x) > 0.002)
        return sin(x) / x;
    return 1 - x * x / 6 * (1 - x * x / 20);
}

VisiLibity::Point circline(double x, double y, double th, double s, double k){
    double sin, cos;
    sincos(th + k * s / 2., &sin, &cos);
    return {
        x + s * sinc(k * s / 2.) * cos,
        y + s * sinc(k * s / 2.) * sin,
    };
}

path_d sample_arc(dubins::d_arc arc, uint_fast32_t n_samples = 30){
    path_d v{ n_samples + 1 };

    // &p - &v[0] is a nice hack to get the element idx without race conditions
    std::for_each(v.begin(), v.end(), [&](auto& p){
        double s = arc.L / n_samples * (&p - &v[0]);
        p = circline(arc.x0, arc.y0, arc.th0, s, arc.k);
    });

    return v;
}

void arc_dump(auto path){
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << std::fixed << "\\operatorname{polygon}((" << path[i].x() << ", " << path[i].y() << "), (";
        std::cout << std::fixed << path[i + 1].x() << ", " << path[i + 1].y() << "))\n";
    }
    std::cout << "---------------------------------------------\n";
}

void dubins_dump(const auto& dpath){
    if(dpath.size() >= 500)
        return;
    std::cout << "Dubins path:\n";
    for (const auto& c : dpath){
        arc_dump(sample_arc(c.a1, 5));
        arc_dump(sample_arc(c.a2, 5));
        arc_dump(sample_arc(c.a3, 5));
    }
    std::cout << "---------------------------------------------\n";
}
*/

/**
 * @brief DPS algorithm to get a solution piece given three points of the polyline
 * 
 * @param a starting point
 * @param b middle point
 * @param c ending point
 * @param new_a arriving point of the solution piece (q2)
 * @param r turning radius
 * @return the solution piece as a dubins curve
 */
dubins::d_curve get_safe_curve(VisiLibity::Point a, VisiLibity::Point b, VisiLibity::Point c, VisiLibity::Point& new_a, double r){

    dubins::d_curve curve;

    double vx0 = b.x() - a.x();
    double vy0 = b.y() - a.y();
    double th0 = atan2(vy0, vx0); // * sgn(vy0);

    double norm0 = sqrt(vx0 * vx0 + vy0 * vy0);
    double unitx0 = vx0 / norm0;
    double unity0 = vy0 / norm0;

    double vxf = c.x() - b.x();
    double vyf = c.y() - b.y();
    double thf = atan2(vyf, vxf); // * sgn(vyf);

    double normf = sqrt(vxf * vxf + vyf * vyf);
    double unitxf = vxf / normf;
    double unityf = vyf / normf;

    /*
     * |A·B| = |A| |B| cos(θ)
     * |A×B| = |A| |B| sin(θ)
     * with this we can easily get the angle between the two vectors
     * we add fabs to normalize in [0, pi)
     */

    //TODO atan2 vs pi-atan2 && sin/cos vs cos/sin
    double cross_prod_3_component = vx0 * vyf - vy0 * vxf;
    double abs_cross_prod = fabs(cross_prod_3_component);
    double alpha = atan2(abs_cross_prod, vx0 * vxf + vy0 * vyf); //angle between vectors
    double d = r * (abs_cross_prod / (vx0 * vxf + vy0 * vyf + normf * norm0));
    double xf = b.x() + d * unitxf;
    double yf = b.y() + d * unityf;
    new_a.set_x(xf);
    new_a.set_y(yf);

    // this is slower than actually creating the curve manually
    // [[maybe_unused]] int ok = dubins::d_shortest(a.x(), a.y(), th0, xf, yf, thf, 1. / r, curve);

    VisiLibity::Point turning_point{b.x() + d * -unitx0, b.y() + d * -unity0};
    double straight_segment_len = sqrt((turning_point.x() - a.x())*(turning_point.x() - a.x()) + (turning_point.y() - a.y())*(turning_point.y() - a.y()));

    curve = {
        .a1 = {a.x(), a.y(), th0, 0, 0, a.x(), a.y(), th0}, // garbage
        .a2 = {a.x(), a.y(), th0, 0, straight_segment_len, turning_point.x(), turning_point.y(), th0},
        .a3 = {turning_point.x(), turning_point.y(), th0, ((cross_prod_3_component > 0) - (cross_prod_3_component < 0)) / r, alpha * r, xf, yf, thf},
        .L = straight_segment_len + alpha * r
    };

    return curve;
}

/**
 * @brief helper function of the test to handle last solution piece
 */
double dubins_wrapper(const VisiLibity::Polyline& path, multi_dubins::path_t& sol, VisiLibity::Point& new_a, double r){
    new_a = path[0];

    auto start_time = std::chrono::system_clock::now();

    for (uint64_t i = 0; i < path.size() - 2; ++i)
        sol[i] = get_safe_curve(new_a, path[i + 1], path[i + 2], new_a, r);

    // create the last trait (straight segment)
    double x_prev = new_a.x();
    double y_prev = new_a.y();
    double th_prev = sol[sol.size() - 2].a3.thf; 

    double x_final = path[path.size() - 1].x();
    double y_final = path[path.size() - 1].y();
    // th_final == th_prev

    double dist = sqrt((x_prev - x_final) * (x_prev - x_final) + (y_prev - y_final) * (y_prev - y_final));

    sol.back() = {
        .a1 = {x_prev, y_prev, th_prev, 0, 0, x_prev, y_prev, th_prev},
        .a2 = {x_prev, y_prev, th_prev, 0, dist, x_final, y_final, th_prev},
        .a3 = {x_prev, y_prev, th_prev, 0, 0, x_final, y_final, th_prev},
        .L = dist
    };
        
    auto end_time = std::chrono::system_clock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
}

/**
 * @brief helper function
 */
multi_dubins::path_t dubins_path(const VisiLibity::Polyline& path, double th0, double thf, double r, double& time){

    // general case
    multi_dubins::path_t sol{ path.size() - 1 };
    VisiLibity::Point new_a; // useless here, backward compatibility with version with no first and last curves

    time = dubins_wrapper(path, sol, new_a, r);
    
    return sol;
}

/**
 * @brief first test, interpolation using DPS
 * 
 * @param shortest_path the polyline
 * @param [out] t elapsed time
 * @param [out] l length of the solution
 */
void test(const VisiLibity::Polyline& shortest_path, double& t, double& l){
    double th0 = atan2(shortest_path[1].y() - shortest_path[0].y(), shortest_path[1].x() - shortest_path[0].x());
    double thf = atan2(shortest_path[shortest_path.size() - 1].y() - shortest_path[shortest_path.size() - 2].y(), 
        shortest_path[shortest_path.size() - 1].x() - shortest_path[shortest_path.size() - 2].x());
    
    double time;

    // get the path made out of dubin curves
    multi_dubins::path_t p0 = dubins_path(shortest_path, th0, thf, .4, time);
    
    double len = 0;
    for(int i = 0; i < p0.size(); ++i)
        len += p0[i].L;
    
    // dubins_dump(p0);
    // std::cout << std::fixed << "time elapsed: " << time << " us Total length: " << len << "\n";

    t = time;
    l = len;
}

/**
 * @brief DPS algorithm to get a solution piece given three points of the polyline
 * this is implemented not with the closed form solution but by computing the dubins curve from
 * a to q2 with the appropriate angles
 * 
 * @param a starting point
 * @param b middle point
 * @param c ending point
 * @param new_a arriving point of the solution piece (q2)
 * @param r turning radius
 * @return the solution piece as a dubins curve
 */
dubins::d_curve get_safe_curve1(VisiLibity::Point a, VisiLibity::Point b, VisiLibity::Point c, VisiLibity::Point& new_a, double r){

    dubins::d_curve curve;

    /*
    std::cerr << a.x() << " " << a.y() << '\n';
    std::cerr << b.x() << " " << b.y() << '\n';
    std::cerr << c.x() << " " << c.y() << '\n';
    */

    double vx0 = b.x() - a.x();
    double vy0 = b.y() - a.y();
    double th0 = atan2(vy0, vx0); // * sgn(vy0);

    /*
    double norm0 = sqrt(vx0 * vx0 + vy0 * vy0);
    double unitx0 = vx0 / norm0;
    double unity0 = vy0 / norm0;
    */

    double vxf = c.x() - b.x();
    double vyf = c.y() - b.y();
    double thf = atan2(vyf, vxf); // * sgn(vyf);

    double normf = sqrt(vxf * vxf + vyf * vyf);
    double unitxf = vxf / normf;
    double unityf = vyf / normf;

    /*
     * |A·B| = |A| |B| cos(θ)
     * |A×B| = |A| |B| sin(θ)
     * with this we can easily get the angle between the two vectors
     * we add fabs to normalize in [0, pi)
     */

    // TODO atan2 vs pi-atan2 && sin/cos vs cos/sin
    double alpha = M_PI - atan2(fabs(vx0 * vyf - vy0 * vxf), vx0 * vxf + vy0 * vyf);
    double sina, cosa;
    sincos(alpha / 2., &sina, &cosa);
    double d = r * (cosa / sina);
    double xf = b.x() + d * unitxf;
    double yf = b.y() + d * unityf;
    new_a.set_x(xf);
    new_a.set_y(yf);

    // this is slower than actually creating the curve manually
    [[maybe_unused]] int ok = dubins::d_shortest(a.x(), a.y(), th0, xf, yf, thf, 1. / r, curve);

    return curve;
}

/**
 * @brief helper function of the test to handle last solution piece
 */
double dubins_wrapper1(const VisiLibity::Polyline& path, multi_dubins::path_t& sol, VisiLibity::Point& new_a, double r){
    new_a = path[0];

    auto start_time = std::chrono::system_clock::now();

    for (uint64_t i = 0; i < path.size() - 2; ++i){
        sol[i] = get_safe_curve1(new_a, path[i + 1], path[i + 2], new_a, r);
        // std::cerr << new_a.x() << " " << new_a.y() << '\n';
    }

    // create the last trait (straight segment)
    double x_prev = new_a.x();
    double y_prev = new_a.y();
    double th_prev = sol[sol.size() - 2].a3.thf; 

    double x_final = path[path.size() - 1].x();
    double y_final = path[path.size() - 1].y();
    // th_final == th_prev 

    double dist = sqrt((x_prev - x_final) * (x_prev - x_final) + (y_prev - y_final) * (y_prev - y_final));

    sol.back() = {
        .a1 = {x_prev, y_prev, th_prev, 0, 0, x_prev, y_prev, th_prev},
        .a2 = {x_prev, y_prev, th_prev, 0, dist, x_final, y_final, th_prev},
        .a3 = {x_prev, y_prev, th_prev, 0, 0, x_final, y_final, th_prev},
        .L = dist
    };
        
    auto end_time = std::chrono::system_clock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
}

/**
 * @brief helper function
 */
multi_dubins::path_t dubins_path1(const VisiLibity::Polyline& path, double th0, double thf, double r, double& time){

    // general case
    multi_dubins::path_t sol{ path.size() - 1 };
    VisiLibity::Point new_a; // useless here, backward compatibility with version with no first and last curves

    time = dubins_wrapper1(path, sol, new_a, r);
    
    return sol;
}

/**
 * @brief second test, interpolation using DPS + dubins
 * 
 * @param shortest_path the polyline
 * @param [out] t elapsed time
 * @param [out] l length of the solution
 */
void test1(const VisiLibity::Polyline& shortest_path, double& t, double& l){
    double th0 = atan2(shortest_path[1].y() - shortest_path[0].y(), shortest_path[1].x() - shortest_path[0].x());
    double thf = atan2(shortest_path[shortest_path.size() - 1].y() - shortest_path[shortest_path.size() - 2].y(), 
        shortest_path[shortest_path.size() - 1].x() - shortest_path[shortest_path.size() - 2].x());
    
    double time;

    // get the path made out of dubin curves
    multi_dubins::path_t p0 = dubins_path1(shortest_path, th0, thf, .4, time);
    
    double len = 0;
    for(int i = 0; i < p0.size(); ++i)
        len += p0[i].L;
    
    // dubins_dump(p0);
    // std::cout << std::fixed << "time elapsed: " << time << " us Total length: " << len << "\n";

    t = time;
    l = len;
}

/**
 * @brief offsetts the environment using Clipper2
 */
Clipper2Lib::PathsD offset_env(const Clipper2Lib::PathsD& border, const Clipper2Lib::PathsD& ob, [[maybe_unused]] double robot_r, double min_r){
    auto border_off = Clipper2Lib::InflatePaths(border, -.4, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
    auto ob_off = Clipper2Lib::InflatePaths(ob, min_r, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon, 2.1);
    Clipper2Lib::ClipperD cd;
    Clipper2Lib::PathsD p;
    cd.AddClip(ob_off);
    cd.AddSubject(border_off);
    cd.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, p);

    // we need to do this crap cause for some reason VisiLibity cannot stand obstacles sharing a vertex with the border!?!?
    cd.Clear();
    Clipper2Lib::PathsD border_aux = { p[0] };
    p.erase(p.begin());
    cd.AddClip(border_aux);
    cd.AddSubject(p);
    cd.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, p);
    p.emplace(p.begin(), border_aux[0].rbegin(), border_aux[0].rend());

    return p;
}

/*
void desmos_dump(auto clipper_env){
    for (const auto& s : clipper_env){
        std::cout << "\\operatorname{polygon}(";
        for (const auto t : s)
            std::cout << "(" << t.x << ", " << t.y << "), ";
        std::cout << "\b\b)\n";
    }
}

void polyline_dump(const auto& p){
    for (auto i = 0U; i < p.size() - 1; ++i){
        std::cout << "\\operatorname{polygon}(";
        std::cout << "(" << p[i].x() << ", " << p[i].y() << "), ";
        std::cout << "(" << p[i + 1].x() << ", " << p[i + 1].y() << "))\n";
    }
}
*/

/**
 * @brief third test, interpolation of the polyline using MPDP
 * 
 * @param shortest_path the polyline
 * @param [out] t elapsed time
 * @param [out] l length of the solution
 * @note the resulting path is assumed to be collision free, even though MPDP has no such guarantees
 */
void test3(const VisiLibity::Polyline& shortest_path, double& t, double& l){
    double th0 = atan2(shortest_path[1].y() - shortest_path[0].y(), shortest_path[1].x() - shortest_path[0].x());
    double thf = atan2(shortest_path[shortest_path.size() - 1].y() - shortest_path[shortest_path.size() - 2].y(), 
        shortest_path[shortest_path.size() - 1].x() - shortest_path[shortest_path.size() - 2].x());

    std::vector<Configuration2> my_test;
    my_test.push_back({shortest_path[0].x(), shortest_path[0].y(), th0});

    for (auto i = 1UL; i < shortest_path.size() - 1; ++i)
        my_test.push_back({shortest_path[i].x(), shortest_path[i].y(), ANGLE::FREE});

    my_test.push_back({shortest_path[shortest_path.size() - 1].x(), shortest_path[shortest_path.size() - 1].y(), th0});

    std::vector<bool> fixedAngles(shortest_path.size(), false); // ???
    fixedAngles.front() = true;
    fixedAngles.back() = true;

    std::vector<real_type> curveParam={1 / .4};

    auto start_time = std::chrono::system_clock::now();
    std::pair<LEN_T, std::vector<Angle>> ret = DP().solveDP(my_test, fixedAngles, curveParam, 4, 8); // xdddddd
    auto end_time = std::chrono::system_clock::now();
    t = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();

    LEN_T ComLength = ret.first;
    std::vector<Angle> vtheta = ret.second;

    LEN_T Length = 0;
    for (unsigned int i = my_test.size() - 1; i > 0; i--){
        my_test[i - 1].th(vtheta[i - 1]);
        my_test[i].th(vtheta[i]);
        Dubins c(my_test[i - 1], my_test[i], 1 / .4);
        Length += c.l();
    }

    // std::cerr << ComLength << " " << Length << '\n';

    l = Length;
}

int main(int argc, char** argv){
    double e = std::atof(argv[1]);
    uint32_t n;
    std::cin >> n;

    Clipper2Lib::PathsD clipper_border{1};
    Clipper2Lib::PathD clipper_border_aux;
    Clipper2Lib::PathsD clipper_obs{n - 1};

    double a0, b0, c0, d0, a1, b1, c1, d1;
    int m;
    std::cin >> m;
    for (int i = 0; i < m; ++i){
        double x, y;
        std::cin >> x >> y;
        clipper_border_aux.emplace_back(x, y);
    }
    clipper_border[0] = std::move(clipper_border_aux);

    for (auto i = 1U; i < n; ++i){
        Clipper2Lib::PathD polygon(4);
        std::cin >> a0 >> a1 >> b0 >> b1 >> c0 >> c1 >> d0 >> d1;
        polygon[0] = {a0, a1};
        polygon[1] = {b0, b1};
        polygon[2] = {c0, c1};
        polygon[3] = {d0, d1};
        clipper_obs[i - 1] = std::move(polygon);
    }

    auto aux = offset_env(clipper_border, clipper_obs, .45, .45);
    // desmos_dump(clipper_border);
    // desmos_dump(clipper_obs);
    //exit(1);
    // desmos_dump(aux);

    std::vector<VisiLibity::Polygon> visilibity_env(aux.size());
    std::transform(std::execution::par_unseq, aux.begin(), aux.end(), visilibity_env.begin(), [](const auto& p){
        VisiLibity::Polygon ptmp;
        for (auto i = p.rbegin(); i != p.rend(); ++i)
            ptmp.push_back(VisiLibity::Point(i->x, i->y));
        return ptmp;
    });
    VisiLibity::Environment env_ = VisiLibity::Environment(visilibity_env);

    assert(env_.is_valid(e));

    VisiLibity::Visibility_Graph graph(env_, e);

    auto pts = env_.random_points(20000, e);
    
    std::cout << "N,A* (us),PD (us),PD (len),Dubins (us),Dubins (len),mpdp (us),mpdp (len)\n";
    // VisiLibity::Polyline shortp;
    for (auto i = 0UL; i < pts.size(); i += 2){
        auto start0 = std::chrono::high_resolution_clock::now();
        const auto shortp = env_.shortest_path(pts[i], pts[i + 1], graph, e);
        auto end0 = std::chrono::high_resolution_clock::now();
        double time0 = std::chrono::duration_cast<std::chrono::nanoseconds>(end0 - start0).count();

        auto N = shortp.size();

        double time1, len1;
        test(shortp, time1, len1);

        double time2, len2;
        test1(shortp, time2, len2);

        double time3, len3;
        test3(shortp, time3, len3);

        std::cout << N << "," << time0 << "," << time1 << "," << len1 << "," << time2 << "," << len2 << "," << time3 << "," << len3 << "\n"; 
    }
}