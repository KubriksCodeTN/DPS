/**
 * @file dubins_seq_ompl.cpp
 * @brief DPS interpolation using OMPL's Dubins curve implementation 
 */
#include "planner.hpp"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <boost/program_options.hpp>

namespace ob = ompl::base;
// namespace og = ompl::geometric;
namespace po = boost::program_options;

ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(.5));

/**
 * @brief generates the solution piece
 * 
 * @param a starting point
 * @param b intersection point of the two edges
 * @param c ending point of the third edge
 * @param [out] new_a ending point of dubins curve
 * @param r minmum curvature radius
 * @return the solution piece
 */
dubins::d_curve Planner::get_safe_curve(VisiLibity::Point a, VisiLibity::Point b, VisiLibity::Point c, VisiLibity::Point& new_a, double r){

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


    // -------- OMPL test -----------



    ob::ScopedState<> start(space), goal(space);

    ob::RealVectorBounds bounds(2);
    double largeValue = 99999999;//std::numeric_limits<double>::max() / 4;
    bounds.setLow(-largeValue);
    bounds.setHigh(largeValue);

    space->as<ob::DubinsStateSpace>()->setBounds(bounds);

    start[0] = a.x();
    start[1] = a.y();
    start[2] = th0;

    goal[0] = xf;
    goal[1] = yf;
    goal[2] = thf;

    double l_temp = space->distance(start.get(), goal.get());

    if(std::fabs(l_temp) < 1.e-3){
        std::cerr << "Unable to interpolate the polyline\n";
        exit(18);
    }

    curve = {
        .a1 = {0}, // garbage
        .a2 = {0},
        .a3 = {0, 0, 0, 0, l_temp, 0,0,0},
        .L = l_temp
    };

    return curve;
    
    
    // ------------------------------

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
 * @brief helper function
 */
double Planner::dubins_wrapper(const VisiLibity::Polyline& path, multi_dubins::path_t& sol, VisiLibity::Point& new_a, double r){
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

    return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
}