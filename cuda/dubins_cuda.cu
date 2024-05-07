#include "../planner.hpp"
#include "../multi_dubins/multi_dubins.hpp"
#include "cuda_runtime.h"

struct point_t{
    double x,y;
};

__global__ void get_safe_curve_cuda(
    double* x_components,
    double* y_components, 
    point_t* new_a_arr,
    dubins::d_curve* out_arr,
    const double r,
    const int N_points
){

    int id = blockIdx.x * blockDim.x + threadIdx.x;
    if(id > N_points - 3)
        return;

    point_t a = {x_components[id], y_components[id]};
    point_t b = {x_components[id + 1], y_components[id + 1]};
    point_t c = {x_components[id + 2], y_components[id + 2]};

    double vx0 = b.x - a.x;
    double vy0 = b.y - a.y;
    double th0 = atan2(vy0, vx0); // * sgn(vy0);

    double norm0 = sqrt(vx0 * vx0 + vy0 * vy0);
    double unitx0 = vx0 / norm0;
    double unity0 = vy0 / norm0;

    double vxf = c.x - b.x;
    double vyf = c.y - b.y;
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
    
    //old version with sincos
/*   
    double cross_prod_3_component = vx0 * vyf - vy0 * vxf;
    double alpha = M_PI - atan2(fabs(cross_prod_3_component), vx0 * vxf + vy0 * vyf); //angle between vectors
    double sina, cosa;
    sincos(alpha / 2., &sina, &cosa);
    double d = r * (cosa / sina);
    double xf = b.x + d * unitxf;
    double yf = b.y + d * unityf;
    new_a_arr[id] = {xf, yf};
*/
    

    double cross_prod_3_component = vx0 * vyf - vy0 * vxf;
    double abs_cross_prod = fabs(cross_prod_3_component);
    double alpha = M_PI - atan2(abs_cross_prod, vx0 * vxf + vy0 * vyf); //angle between vectors "pi -" can be canceled in every place where alpha is used 
    double d = r * (abs_cross_prod / (vx0 * vxf + vy0 * vyf + normf * norm0));
    double xf = b.x + d * unitxf;
    double yf = b.y + d * unityf;
    new_a_arr[id] = {xf, yf};

    point_t turning_point{b.x - d * unitx0, b.y - d * unity0};
    double straight_segment_len = sqrt((turning_point.x - a.x)*(turning_point.x - a.x) + (turning_point.y - a.y)*(turning_point.y - a.y));

    dubins::d_curve curve = {
        .a1 = {a.x, a.y, th0, 0, 0, a.x, a.y, th0}, // garbage
        .a2 = {a.x, a.y, th0, 0, straight_segment_len, turning_point.x, turning_point.y, th0},
        .a3 = {turning_point.x, turning_point.y, th0, ((cross_prod_3_component > 0) - (cross_prod_3_component < 0)) / r, (M_PI - alpha) * r, xf, yf, thf},
        .L = /* a1.len (which is 0) + */ straight_segment_len + (M_PI - alpha) * r
    };

    out_arr[id] = curve;

    // correct the final point and length of each curve
    // __syncthreads();

    // if(id != N_points - 3){
    //     out_arr[id + 1].a2.x0 = xf;
    //     out_arr[id + 1].a2.y0 = yf;
    //     out_arr[id + 1].a2.L = sqrt((xf - out_arr[id + 1].a2.xf) * (xf - out_arr[id + 1].a2.xf) + 
    //         (yf - out_arr[id + 1].a2.yf) * (yf - out_arr[id + 1].a2.yf));   
    //     out_arr[id + 1].L = out_arr[id + 1].a2.L + out_arr[id + 1].a3.L;
    // }    
}

double Planner::dubins_wrapper(const VisiLibity::Polyline& path, multi_dubins::path_t& sol, VisiLibity::Point& new_a, double r){
    std::vector<double> x_components_h, y_components_h;
    std::vector<point_t> new_a_arr_h{ sol.size() - 2 }; // first and last curves dont generate a new_a point
    int threads = 32;
    int blocks = (sol.size() + threads - 1) / threads; 

    for (uint64_t i = 1; i < path.size(); ++i){
        x_components_h.push_back(path[i].x());
        y_components_h.push_back(path[i].y());
    } 
    
    int n_bytes_x_components = sizeof(double) * x_components_h.size();
    int n_bytes_out_arr = sizeof(dubins::d_curve) * new_a_arr_h.size(); // n. of curves = n. of new_a points
    int n_bytes_new_a_arr = sizeof(point_t) * new_a_arr_h.size();

    double *x_components, *y_components;
    cudaMalloc(&x_components, n_bytes_x_components);
    cudaMalloc(&y_components, n_bytes_x_components);

    point_t *new_a_arr_dev;
    cudaMalloc(&new_a_arr_dev, n_bytes_new_a_arr);

    dubins::d_curve* out_arr_dev;
    cudaMalloc(&out_arr_dev, n_bytes_out_arr);

    cudaMemcpy(x_components, x_components_h.data(), n_bytes_x_components, cudaMemcpyHostToDevice);
    cudaMemcpy(y_components, y_components_h.data(), n_bytes_x_components, cudaMemcpyHostToDevice);

    auto start_time = std::chrono::system_clock::now();
    get_safe_curve_cuda<<<blocks, threads>>>(x_components, y_components, new_a_arr_dev, out_arr_dev, r, x_components_h.size());
    auto end_time = std::chrono::system_clock::now();
    
    //std::cout << cudaDeviceSynchronize()<<"\n";

    cudaMemcpy(sol.data()+1, out_arr_dev, n_bytes_out_arr, cudaMemcpyDeviceToHost); // +1 leaves space for 1° curve
    cudaMemcpy(new_a_arr_h.data(), new_a_arr_dev, n_bytes_new_a_arr, cudaMemcpyDeviceToHost);

    // adjust the 
    for(int i = 2; i < sol.size()-1; ++i){

        sol[i].a2.x0 = new_a_arr_h[i-2].x;
        sol[i].a2.y0 = new_a_arr_h[i-2].y;

        // I want to have a continous curve
        sol[i].a1.x0 = sol[i].a2.x0;
        sol[i].a1.y0 = sol[i].a2.y0;
        sol[i].a1.xf = sol[i].a2.x0;
        sol[i].a1.yf = sol[i].a2.y0;

        sol[i].a2.L = sqrt(((sol[i].a2.xf - sol[i].a2.x0)*(sol[i].a2.xf - sol[i].a2.x0) + 
            (sol[i].a2.yf - sol[i].a2.y0)*(sol[i].a2.yf - sol[i].a2.y0)));
        sol[i].L = sol[i].a2.L + sol[i].a3.L;;
    }

    // needed to adjust the length of the curve between second and third point
    // can't do it in CUDA since out_arr[0] is not corrected after __syncthreads()
    // sol[1].a2.L = sqrt(((sol[1].a2.xf - sol[1].a2.x0)*(sol[1].a2.xf - sol[1].a2.x0) + 
    //         (sol[1].a2.yf - sol[1].a2.y0)*(sol[1].a2.yf - sol[1].a2.y0)));
    // sol[1].L += sol[1].a2.L;

    new_a = {new_a_arr_h.back().x, new_a_arr_h.back().y};

    return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
}