/*
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>
*/

/*
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
*/

/**
 * @file main_2d.cpp
 * @brief Main of the rrt*-PD test
 */

#include <iostream>
#include "rrtstar.hpp"

// yes, this is garbage
using namespace rrtstar;

/**
 * @brief Euclidean distance
 */
inline double L2(const point_t& a, const point_t& b){
    return sqrt((b.get<0>() - a.get<0>()) * (b.get<0>() - a.get<0>()) + (b.get<1>() - a.get<1>()) * (b.get<1>() - a.get<1>()));
}

/**
 * @brief calculates the length of the solution piece found
 * 
 * @param a starting point
 * @param b middle point
 * @param c ending point
 * @param [out] new_a ending point of the solution piece
 * @param r minmum curvature radius
 * @return the length of the solution piece
 */
double get_safe_curve_len(point_t a, point_t b, point_t c, [[maybe_unused]] point_t& new_a,  double r){
    double L;

    double vx0 = b.get<0>() - a.get<0>();
    double vy0 = b.get<1>() - a.get<1>();
    // double th0 = atan2(vy0, vx0); // * sgn(vy0);

    double norm0 = sqrt(vx0 * vx0 + vy0 * vy0);
    double unitx0 = vx0 / norm0;
    double unity0 = vy0 / norm0;

    double vxf = c.get<0>() - b.get<0>();
    double vyf = c.get<1>() - b.get<1>();
    // double thf = atan2(vyf, vxf); // * sgn(vyf);

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
    double xf = b.get<0>() + d * unitxf;
    double yf = b.get<1>() + d * unityf;
    new_a = {xf, yf};

    // this is slower than actually creating the curve manually
    // [[maybe_unused]] int ok = dubins::d_shortest(a.x(), a.y(), th0, xf, yf, thf, 1. / r, curve);

    point_t turning_point{b.get<0>() + d * -unitx0, b.get<1>() + d * -unity0};
    double straight_segment_len = sqrt((turning_point.get<0>() - a.get<0>()) * (turning_point.get<0>() - a.get<0>()) 
        + (turning_point.get<1>() - a.get<1>()) * (turning_point.get<1>() - a.get<1>()));
    
    L = straight_segment_len + alpha * r;

    return L;
}

/**
 * @brief given a leaf of the tree calculate the total length of the solution from the root to the leaf
 */
double get_distance_interpolation(node* x){
    double L = 0;

    std::vector<point_t> v;
    while (x){
        v.push_back(x->p);
        x = x->parent;
    }
    std::reverse(v.begin(), v.end());

    if (v.size() == 2)
        return L2(v[0], v[1]);

    point_t a = v[0];
    point_t b = v[1];

    point_t new_a;

    for (uint64_t i = 2; i < v.size(); ++i){
        L += get_safe_curve_len(a, b, v[i], new_a, .5);
        a = new_a;
        b = v[i];
    }

    L += L2(new_a, v.back());

    return L;
}

/**
 * @brief given a leaf of the tree calculate the total length of the solution from the root to the leaf
 * 
 * @param x leaf node
 * @return INF if x is nullptr else the polyline len
 */
inline double get_distance_1(node* x){
    if (!x) return std::numeric_limits<double>::max();
    return get_distance(x);
}

/**
 * @brief tangent of the angle formed by vector p0p1 and p1p2, always considers the smaller angle
 */
double get_tan_h(point_t p0, point_t p1, point_t p2){
    double vx0 = p1.get<0>() - p0.get<0>();
    double vy0 = p1.get<1>() - p0.get<1>();
    double vxf = p2.get<0>() - p1.get<0>();
    double vyf = p2.get<1>() - p1.get<1>();
    double norm0 = sqrt(vx0 * vx0 + vy0 * vy0);
    double normf = sqrt(vxf * vxf + vyf * vyf);
    // fabs?
    return (fabs(vx0 * vyf - vy0 * vxf) / (vx0 * vxf + vy0 * vyf + normf * norm0));
}

/**
 * @brief checks if the polyline from the leaf n from the root of the tree satisfies the necessary constraints
 */
bool check_ok(node* n){
    constexpr double r = .5;

    node* p = n;

    point_t p0 = p->p;
    p = p->parent;
    point_t p1 = p->p;
    p = p->parent;
    point_t p2 = p->p;

    double oldtan = get_tan_h(p0, p1, p2);

    if (L2(p0, p1) < r * oldtan)
        return false;

    if (!p->parent) return true;

    p = p->parent;
    while (p != nullptr){
        p0 = p1;
        p1 = p2;
        p2 = p->p;

        if (L2(p0, p1) < r * oldtan + r * get_tan_h(p0, p1, p2))
            return false;
        oldtan = get_tan_h(p0, p1, p2);
        p = p->parent;
    }

    if (L2(p2, p1) < r * oldtan)
        return false;

    return true;
}

int main(int argc, char** argv){
    double e = std::stod(argv[1]);
    double stepsz = std::stod(argv[2]);
    double gamma = std::stod(argv[3]);
    double n_iter = std::stod(argv[4]);
    point_t start = {std::stod(argv[5]), std::stod(argv[6])};
    point_t end = {std::stod(argv[7]), std::stod(argv[8])};

    double a0, b0, c0, d0, a1, b1, c1, d1;
    [[maybe_unused]] int m, n;
    rtree r_tree;

    std::cin >> n;
    std::cin >> a0 >> a1 >> c0 >> c1;

    double h = c0, l = c1;

    for (auto i = 1; i < n; ++i){
        std::cin >> a0 >> a1 >> c0 >> c1;
        box_t b({a0, a1}, {c0, c1});
        r_tree.insert(b);
    }

    // 720, 440

    auto res = rrt(r_tree, start, end, h, l, e, stepsz, gamma, n_iter);
    if (!res.size()){
        std::cout << "NA\n";
        return 1;
    } 

    // std::cout << res.size() << '\n';

    node* x = nullptr;

    for (const auto& x1 : res){
        if (get_distance_1(x) > get_distance(x1) && check_ok(x1))
            x = x1;
    } 

    if (!x){
        std::cout << "NA\n";
        return 1;
    } 

    std::cout << get_distance_interpolation(x) << '\n';
    return 0;

    point_t tmp = x->p;
    auto cp = x;
    while (cp){
        std::cout << '(' << tmp.get<0>() << ',' << tmp.get<1>() << ")\n";
        cp = cp->parent;
        if (cp) tmp = cp->p;
    }
    // }
}
