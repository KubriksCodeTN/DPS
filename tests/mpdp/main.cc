#include <dubins.hh>
#include <dp.hh>
#include <timeperf.hh>
#include<tests.hh>

#include<iostream>
#include<fstream>
#include<cmath>
#include<random>
#include<vector>
#include<utility>
#include<iomanip>
#include<algorithm>


// dubins.hpp
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
//------------------
// dubins.cpp
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
inline double mod2pi(double theta){
    return fmodr(theta, 2 * M_PI);
}

/**
 * @brief numerically stable implementation of the sinc function
 */
__attribute__((weak)) double sinc(double x){
    if (fabs(x) > 0.002)
        return sin(x) / x;
    return 1 - x * x / 6 * (1 - x * x / 20);
}


triplet circline(double s, double x0, double y0, double th0, double k){
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
 * 
 * @todo better parameters handling
 */
std_vals to_std(double x0, double y0, double th0, double xf,
              double yf, double thf, double Kmax){
    // std_vals ret;
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    double tmp = hypot(dx, dy) / 2.;
    // ret.lambda = sqrt(dx * dx + dy * dy) / 2;
    // ret.th0 = mod2pi(th0 - phi);
    // ret.thf = mod2pi(thf - phi);
    // ret.Kmax = Kmax * ret.lambda;
    return {mod2pi(th0 - phi), mod2pi(thf - phi), Kmax * tmp, tmp};
}

/**
 * @brief possible solutions to the dubins problem
 * 
 * @todo use sincos function (faster and probably more precise)
 * GCC might be already doing it under the hood tho
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
    std::sort(out.begin(), out.end(), [](const d_curve& a, const d_curve& b){
        return a.L < b.L;
    });
    return out;
}
//----------------------


// debug print
#define __DEBUG
#ifdef __DEBUG

struct point_t{
  double x,y;
};

using path_d = std::vector<point_t>;
path_d sample_arc(dubins::d_arc, uint_fast32_t);

/**
 * @brief creates a dump to visualize a dubins arc on desmos
 */
void arc_dump(const path_d& path){
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << std::fixed << "\\operatorname{polygon}((" << path[i].x << ", " << path[i].y << "), (";
        std::cout << std::fixed << path[i + 1].x << ", " << path[i + 1].y << "))\r\n";
    }
    std::cout << "---------------------------------------------\n";
}

void dubins_dump(const std::vector<d_curve>& dpath){
    if(dpath.size() >= 50)
        return;
    std::cout << "Dubins path:\n";
    for (const auto& c : dpath){
        arc_dump(sample_arc(c.a1, 10));
        arc_dump(sample_arc(c.a2, 10));
        arc_dump(sample_arc(c.a3, 10));
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
 * @brief sample a point from a dubins arc
 */
point_t sample_circline(double x, double y, double th, double s, double k){
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
    std::for_each(v.begin(), v.end(), [&](point_t& p){
        double s = arc.L / n_samples * (&p - &v[0]);
        p = sample_circline(arc.x0, arc.y0, arc.th0, s, arc.k);
    });

    return v;
}
//------------------------------

// void PrintScientific1D(real_type d){
//   if (d == 0)
//   {
//     printf ("%*d", 6, 0);
//     return;
//   }

//   int exponent  = (int)floor(log10( fabs(d)));  // This will round down the exponent
//   real_type base   = d * pow(10, -1.0*exponent);

//   printf("%1.1lfe%+01d", base, exponent);
// }

// void PrintScientific2D(real_type d){
//   if (d == 0)
//   {
//     printf ("%*d", 7, 0);
//     return;
//   }

//   int exponent  = (int)floor(log10( fabs(d)));  // This will round down the exponent
//   real_type base   = d * pow(10, -1.0*exponent);

//   printf("%1.1lfe%+02d", base, exponent);
// }


std::vector<Configuration2> example1 = {
  Configuration2(0,0,-2.0*m_pi/8.0),
  Configuration2(2,2,ANGLE::FREE),
  Configuration2(6,-1,ANGLE::FREE),
  Configuration2(8,1,2.0*m_pi/8.0)
};



std::vector<std::string> testsNames = { 
  "my_test"
}; 

// std::vector<Configuration2> my_test;

// std::vector<std::vector<Configuration2> > Tests = {
//   my_test
// };

std::vector<K_T> Ks = {2.0};
std::vector<uint> discrs = {4};
std::vector<uint> refins = {8};
std::vector<LEN_T> exampleLenghts={3.41557885807514871601142658619, 6.27803455030931356617429628386, 11.9162126542854860389297755319, 7.46756219733842652175326293218, 41.0725016438839318766440555919, 6988.66098639942993031581863761}; //the last length is SPA

std::string nameTest(std::string name, std::string add="", std::string conc=" "){
  if (add==""){
    return name;
  }
  else{
    return name+conc+add;
  }
}

void print_polyline(const std::vector<Configuration2>& path){
    if(path.size() >= 50)
        return;
    std::cout << "Polyline:\n";
    for (uint32_t i = 0U; i < path.size() - 1; ++i){
        std::cout << std::fixed << "\\operatorname{polygon}((" << path[i].x() << ", " << path[i].y() << "), (";
        std::cout << std::fixed << path[i + 1].x() << ", " << path[i + 1].y() << "))\n";
    }
    std::cout << "---------------------------------------------\n";
}


int main(int argc, char** argv){

    std::vector<Configuration2> my_test;

    std::vector< std::vector<Configuration2> > Tests;
    
    uint64_t n_points;
    double x,y,th;
    
    std::cin >> n_points;

    std::cin >> x;
    std::cin >> y;
    std::cin >> th;
    my_test.push_back({x, y, th});
    n_points--;
    
    while(n_points-- > 1){
        //fscanf(stdin, "%lf, %lf, %lf\n", &x, &y, &th);
        std::cin >> x;
        std::cin >> y;
        my_test.push_back({x, y, ANGLE::FREE});
    }

    std::cin >> x;
    std::cin >> y;
    std::cin >> th;
    my_test.push_back({x, y, th});

  bool print_debug = (argc == 2 && std::string(argv[1]) == "print_debug");
    
    Tests.push_back(std::move(my_test));

  for (uint testID=0; testID<Tests.size(); testID++){
    //if (testID!=0){continue;}
    real_type dLen=exampleLenghts[testID];

    std::vector<bool> fixedAngles;
    for (uint i=0; i<Tests[testID].size(); i++){
      if (i==0 || i==Tests[testID].size()-1) {
        fixedAngles.push_back(true);
      }
      else {
        fixedAngles.push_back(false);
      }
    }
    std::vector<real_type> curveParam={Ks[0]};

    for (auto DISCR :  discrs){
      // if (DISCR!=4){continue;}
      for (auto r : refins){
        // if (r!=4){continue;}
        //std::cout << DISCR << " " << r << " ";
        TimePerf tp, tp1;
        std::vector<Configuration2>points=Tests[testID];
        std::vector<d_curve> dubins_path;
        tp.start();
        std::pair<LEN_T, std::vector<Angle> >ret=DP().solveDP(points, fixedAngles, curveParam, DISCR, r);
        auto time1=tp.getTime();
        LEN_T ComLength=ret.first;
        std::vector<Angle> vtheta=ret.second;

        LEN_T Length=0, len=0;
        for (unsigned int i=points.size()-1; i>0; i--){
          points[i-1].th(vtheta[i-1]);
          points[i].th(vtheta[i]);
          Dubins c(points[i-1], points[i], Ks[0]);


          dubins_path.push_back(d_paths(points[i-1].x(), points[i-1].y(), points[i-1].th(), 
            points[i].x(), points[i].y(), points[i].th(), Ks[0])[0]); // 0 is the shortest
          
          //std::cout << "from " << points[i - 1] << " to " << points[i] << "\n";

          //std::cout << c << std::endl;
          Length+=c.l();
          len += dubins_path.back().L;
        }

        std::cout << "discr: " << DISCR << " | refin: " << r;
        //PrintScientific2D((ComLength-exampleLenghts[testID])*1000.0);
        std::cout << " | time: "<< time1 << " ms | length_mpdp: "<< Length << " | length_dps: " << len << "\n";
        //PrintScientific1D(time1);

        if(print_debug){
          print_polyline(Tests[testID]);
          dubins_dump(dubins_path);
        }  
      }
    }
  }
  
  return 0;
}
