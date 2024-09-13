/**
 * @file dubins.cpp
 * @brief implementation of Dubins curves
 */
#include "dubins.hpp"

using namespace dubins;

// TODO use
constexpr double EPSILON = 1e-10;

struct std_vals{
    double th0; ///< starting angle
    double thf; ///< arriving angle
    double Kmax; ///< maximum curvature
    double lambda;
};

struct ls{ ///< stands for lengths
    double s1;
    double s2;
    double s3;
};
typedef ls triplet;

/**
 * @brief mod operation for floating point values
 */
constexpr inline double fmodr(double x, double y){
    return x - y * floor(x / y);
}

/**
 * @brief to get angles in the [0, 2pi) range
 */
constexpr inline double mod2pi(double theta){
    return fmodr(theta, 2 * M_PI);
}

/**
 * @brief numerically stable implementation of the sinc function
 */
constexpr double sinc(double x){
    if (fabs(x) > 0.002)
        return sin(x) / x;
    return 1 - x * x / 6 * (1 - x * x / 20);
}


constexpr triplet circline(double s, double x0, double y0, double th0, double k){
    // ls ret;
    // ret.s1 = x0 + s * sinc(k * s / 2.) * cos(th0 + k * s / 2.);
    // ret.s2 = y0 + s * sinc(k * s / 2.) * sin(th0 + k * s / 2.);
    // ret.s3 = mod2pi(th0 + k * s);
    // return ret;
    return {
        x0 + s * sinc(k * s / 2.) * cos(th0 + k * s / 2.),
        y0 + s * sinc(k * s / 2.) * sin(th0 + k * s / 2.),
        mod2pi(th0 + k * s)
    };
}
/**
 * @brief converts the given problem in standard form
 */
constexpr std_vals to_std(double x0, double y0, double th0, double xf,
              double yf, double thf, double Kmax){
    // std_vals ret;
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    double tmp = sqrt(dx * dx + dy * dy) / 2.;
    // ret.lambda = sqrt(dx * dx + dy * dy) / 2;
    // ret.th0 = mod2pi(th0 - phi);
    // ret.thf = mod2pi(thf - phi);
    // ret.Kmax = Kmax * ret.lambda;
    return {mod2pi(th0 - phi), mod2pi(thf - phi), Kmax * tmp, tmp};
}

/**
 * @brief possible solutions to the dubins problem
 */
///@{
bool LSL(double th0, double thf, double Kmax, ls& aux){
    double tmp2 = 2 + 4 * Kmax * Kmax - 2 * cos(th0 - thf) + 
        4 * Kmax * (sin(th0) - sin(thf));
    if (tmp2 < 0)
        return false;

    double invK = 1. / Kmax;
    double C = cos(thf) - cos(th0);
    double S = 2. * Kmax + sin(th0) - sin(thf);
    double tmp1 = atan2(C, S);
    aux.s1 = invK * mod2pi(tmp1 - th0);
    aux.s2 = invK * sqrt(tmp2);
    aux.s3 = invK * mod2pi(thf - tmp1);
    return true;
}

bool RSR(double th0, double thf, double Kmax, ls& aux){
    double tmp2 = 2 + 4 * Kmax * Kmax - 2 * cos(th0 - thf) - 
        4 * Kmax * (sin(th0) - sin(thf));
    if (tmp2 < 0)
        return false;

    double invK = 1. / Kmax;
    double C = cos(th0) - cos(thf);
    double S = 2 * Kmax - sin(th0) + sin(thf);
    double tmp1 = atan2(C, S);
    aux.s1 = invK * mod2pi(th0 - tmp1);
    aux.s2 = invK * sqrt(tmp2);
    aux.s3 = invK * mod2pi(tmp1 - thf);
    return true;
}

bool LSR(double th0, double thf, double Kmax, ls& aux){
    double tmp3 = 4 * Kmax * Kmax - 2 + 2 * cos(th0 - thf) + 
        4 * Kmax * (sin(th0) + sin(thf));
    if (tmp3 < 0)
        return false;

    double invK = 1 / Kmax;
    double C = cos(th0) + cos(thf);
    double S = 2 * Kmax + sin(th0) + sin(thf);
    double tmp1 = atan2(-C, S);
    aux.s2 = invK * sqrt(tmp3);
    double tmp2 = -atan2(-2, aux.s2 * Kmax);
    aux.s1 = invK * mod2pi(tmp1 + tmp2 - th0);
    aux.s3 = invK * mod2pi(tmp1 + tmp2 - thf);
    return true;
}

bool RSL(double th0, double thf, double Kmax, ls& aux){
    double tmp3 = 4 * Kmax * Kmax - 2 + 2 * cos(th0 - thf) - 
        4 * Kmax * (sin(th0) + sin(thf));
    if (tmp3 < 0)
        return false;

    double invK = 1 / Kmax;
    double C = cos(th0) + cos(thf);
    double S = 2 * Kmax - sin(th0) - sin(thf);
    double tmp1 = atan2(C, S);
    aux.s2 = invK * sqrt(tmp3);
    double tmp2 = atan2(2, aux.s2 * Kmax);
    aux.s1 = invK * mod2pi(th0 - tmp1 + tmp2);
    aux.s3 = invK * mod2pi(thf - tmp1 + tmp2);
    return true;
}

bool RLR(double th0, double thf, double Kmax, ls& aux){
    double tmp2 = 0.125 * (6 - 4 * Kmax * Kmax + 2 * cos(th0 - thf) 
        + 4 * Kmax * (sin(th0) - sin(thf)));
    if (fabs(tmp2) > 1)
        return false;

    double invK = 1 / Kmax;
    double C = cos(th0) - cos(thf);
    double S = 2 * Kmax - sin(th0) + sin(thf);
    double tmp1 = atan2(C, S);
    aux.s2 = invK * mod2pi(2 * M_PI - acos(tmp2));
    aux.s1 = invK * mod2pi(th0 - tmp1 + 0.5 * aux.s2 * Kmax);
    aux.s3 = invK * mod2pi(th0 - thf + Kmax * (aux.s2 - aux.s1));
    return true;
}

bool LRL(double th0, double thf, double Kmax, ls& aux){
    double tmp2 = 0.125 * (6 - 4 * Kmax * Kmax + 2 * cos(th0 - thf) 
        - 4 * Kmax * (sin(th0) - sin(thf)));
    if (fabs(tmp2) > 1)
        return false;

    double invK = 1 / Kmax;
    double C = cos(thf) - cos(th0);
    double S = 2 * Kmax + sin(th0) - sin(thf);
    double tmp1 = atan2(C, S);
    aux.s2 = invK * mod2pi(2 * M_PI - acos(tmp2));
    aux.s1 = invK * mod2pi(tmp1 - th0 + 0.5 * aux.s2 * Kmax);
    aux.s3 = invK * mod2pi(thf - th0 + Kmax * (aux.s2 - aux.s1));
    return true;
}
///@}

/**
 * @brief computes the dubins shortest path from point A to point B given a 
 * starting angle and an arriving angle. Works with maximum curvature constraints
 * 
 * @param [out] aux found solution
 * @return -1 if the path doesn't exist, pid of the function otherwise
 */
int32_t dubins::d_shortest(double x0, double y0, double th0, double xf,
                   double yf, double thf, double Kmax, d_curve& aux){
    const std_vals vals = to_std(x0, y0, th0, xf, yf, thf, Kmax);
    static constexpr std::array<bool(*)(double, double, double, ls&), 6> v =
        {LSL, RSR, LSR, RSL, RLR, LRL};
    // since array 2d is garbage this stays like this
    static constexpr int32_t m[][3] = {
        {1, 0, 1},
        {-1, 0, -1},
        {1, 0, -1},
        {-1, 0, 1},
        {-1, 1, -1},
        {1, -1, 1}
    };

    int32_t pid = -1;
    double s1, s2, s3;
    double L = std::numeric_limits<double>::max();
    ls ls;
    for (auto i = 0UL; i < v.size(); ++i){
        bool tmp = v[i](vals.th0, vals.thf, vals.Kmax, ls);
        double Lcurr = ls.s1 + ls.s2 + ls.s3;
        if (Lcurr >= L || !tmp)
            continue;
        L = Lcurr;
        s1 = ls.s1;
        s2 = ls.s2;
        s3 = ls.s3;
        pid = i;
    }

    if (pid < 0)
        return -1;
    s1 *= vals.lambda;
    s2 *= vals.lambda;
    s3 *= vals.lambda;

    // x0, y0, th0, s1, s2, s3, m[pid][0] * Kmax...
    auto tmp = circline(s1, x0, y0, th0, m[pid][0] * Kmax);
    aux.a1 = {x0, y0, th0, m[pid][0] * Kmax, s1, tmp.s1, tmp.s2, tmp.s3}; 
    auto tmp1 = circline(s2, tmp.s1, tmp.s2, tmp.s3, m[pid][1] * Kmax);
    aux.a2 = {tmp.s1, tmp.s2, tmp.s3, m[pid][1] * Kmax, s2, tmp1.s1, tmp1.s2, tmp1.s3};
    auto tmp2 = circline(s3, tmp1.s1, tmp1.s2, tmp1.s3, m[pid][2] * Kmax);
    aux.a3 = {tmp1.s1, tmp1.s2, tmp1.s3, m[pid][2] * Kmax, s3, tmp2.s1, tmp2.s2, tmp2.s3};
    aux.L = aux.a1.L + aux.a2.L + aux.a3.L;
    // check(...);
    return pid;
}

/**
 * @brief computes the possible dubins paths from point A to point B given a 
 * starting angle and an arriving angle. Works with maximum curvature constraints
 * 
 * @return the vector of admissible paths
 */
std::vector<d_curve> dubins::d_paths(double x0, double y0, double th0, double xf,
                   double yf, double thf, double Kmax){
    const std_vals vals = to_std(x0, y0, th0, xf, yf, thf, Kmax);
    static constexpr std::array<bool(*)(double, double, double, ls&), 6> v =
        {LSL, RSR, LSR, RSL, RLR, LRL};
    // since array 2d is garbage this stays like this
    static constexpr int32_t m[][3] = {
        {1, 0, 1},
        {-1, 0, -1},
        {1, 0, -1},
        {-1, 0, 1},
        {-1, 1, -1},
        {1, -1, 1}
    };

    std::vector<d_curve> out;

    double s1, s2, s3;
    d_curve aux;
    ls ls;

    for (auto i = 0UL; i < v.size(); ++i){
        bool ok = v[i](vals.th0, vals.thf, vals.Kmax, ls);
        if (!ok)
            continue;
        s1 = ls.s1;
        s2 = ls.s2;
        s3 = ls.s3;
        s1 *= vals.lambda;
        s2 *= vals.lambda;
        s3 *= vals.lambda;

        // hack
        const auto& pid = i;
        // x0, y0, th0, s1, s2, s3, m[pid][0] * Kmax...
        auto tmp = circline(s1, x0, y0, th0, m[pid][0] * Kmax);
        aux.a1 = {x0, y0, th0, m[pid][0] * Kmax, s1, tmp.s1, tmp.s2, tmp.s3};
        auto tmp1 = circline(s2, tmp.s1, tmp.s2, tmp.s3, m[pid][1] * Kmax);
        aux.a2 = {tmp.s1, tmp.s2, tmp.s3, m[pid][1] * Kmax, s2, tmp1.s1, tmp1.s2, tmp1.s3};
        auto tmp2 = circline(s3, tmp1.s1, tmp1.s2, tmp1.s3, m[pid][2] * Kmax);
        aux.a3 = {tmp1.s1, tmp1.s2, tmp1.s3, m[pid][2] * Kmax, s3, tmp2.s1, tmp2.s2, tmp2.s3};
        aux.L = aux.a1.L + aux.a2.L + aux.a3.L;

        out.push_back(aux);
    }

    // TODO optimize later
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b){
        return a.L < b.L;
    });
    return out;
}
