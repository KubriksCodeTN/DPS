#include "cuda_runtime.h"
#include "../planner.hpp"
#include <iostream>
#include <chrono>
#include <stdio.h>
#include <cassert>

struct point_t{
    double x, y;
};

/*
struct lk_word{
    point_t p0, q0, q1;
    double th0, thf;
    double k;
};
*/

struct lk_word{
    point_t *p0, *q0, *q1;
    double *th0, *thf;
    double *k;
};

__global__ void warm_up_gpu(){
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    float ia, ib;
    ia = ib = 0.0f;
    ib += ia + tid; 
}

__global__ void get_safe_curve_cuda(
    double* x_components,
    double* y_components, 
    point_t* p0,
    point_t* q0,
    point_t* q1,
    double* th00,
    double* thff,
    double* k,
    const double r,
    const int N_curves
){
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= N_curves) // each thread creates a curve from 3 points a, b, c. the last straigth segment is not created here
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

    double cross_prod_3_component = vx0 * vyf - vy0 * vxf;
    double abs_cross_prod = std::fabs(cross_prod_3_component);
    double d = r * (abs_cross_prod / (vx0 * vxf + vy0 * vyf + normf * norm0));
    double xf = b.x + d * unitxf;
    double yf = b.y + d * unityf;
    double xq0 = b.x - d * unitx0;
    double yq0 = b.y - d * unity0;

    q0[id] = {xq0, yq0};    
    q1[id] = {xf, yf};
    th00[id] = th0;
    thff[id] = thf;
    k[id] = ((cross_prod_3_component > 0) - (cross_prod_3_component < 0)) / r;
    p0[id + 1] = {xf, yf};        
}

double Planner::dubins_wrapper(const VisiLibity::Polyline &path_poly, multi_dubins::path_t &sol, VisiLibity::Point &new_a, double){
    // cuda startup
    cudaFree(0);
    warm_up_gpu<<<1024, 512>>>();

    int32_t n = path_poly.size();

    double threads = 512; // check
    double blocks = (n + threads - 1) / threads;

    double* x;
    double* y;
    double* x_cuda;
    double* y_cuda;
    lk_word path;
    lk_word path_cuda;       

    cudaMallocHost(&x, n * sizeof(double));
    cudaMallocHost(&y, n * sizeof(double));
    cudaMallocHost(&(path.p0), n * sizeof(point_t));
    cudaMallocHost(&(path.q0), n * sizeof(point_t));
    cudaMallocHost(&(path.q1), n * sizeof(point_t));
    cudaMallocHost(&(path.th0), n * sizeof(double));
    cudaMallocHost(&(path.thf), n * sizeof(double));
    cudaMallocHost(&(path.k), n * sizeof(double));
    cudaMalloc(&(path_cuda.p0), n * sizeof(point_t));
    cudaMalloc(&(path_cuda.q0), n * sizeof(point_t));
    cudaMalloc(&(path_cuda.q1), n * sizeof(point_t));
    cudaMalloc(&(path_cuda.th0), n * sizeof(double));
    cudaMalloc(&(path_cuda.thf), n * sizeof(double));
    cudaMalloc(&(path_cuda.k), n * sizeof(double));

    for (auto i = 0; i < n; ++i){
        x[i] = path_poly[i].x();
        y[i] = path_poly[i].y();
        //printf("%lf, %lf\n", x[i], y[i]);
    }

    auto start = std::chrono::high_resolution_clock::now();
    cudaMalloc(&x_cuda, n * sizeof(double));  
    cudaMalloc(&y_cuda, n * sizeof(double));
    cudaMemcpy(x_cuda, x, n * sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(y_cuda, y, n * sizeof(double), cudaMemcpyHostToDevice);
    get_safe_curve_cuda<<<blocks, threads>>>(x_cuda, y_cuda, path_cuda.p0, path_cuda.q0, path_cuda.q1, path_cuda.th0, path_cuda.thf, path_cuda.k,.5, n - 2);   
    cudaMemcpy(path.p0, path_cuda.p0, (n - 2) * sizeof(point_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(path.q0, path_cuda.q0, (n - 2) * sizeof(point_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(path.q1, path_cuda.q1, (n - 2) * sizeof(point_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(path.th0, path_cuda.th0, (n - 2) * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(path.thf, path_cuda.thf, (n - 2) * sizeof(double), cudaMemcpyDeviceToHost);
    cudaMemcpy(path.k, path_cuda.k, (n - 2) * sizeof(double), cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
    auto end = std::chrono::high_resolution_clock::now();

    path.p0[0] = {x[0], y[0]};

    auto dist = [](const point_t& a, const point_t& b){
        return sqrt((a.x - b.x)  * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    };

    // in a3 we use a formula for arc len given radius and coord of points
    // just for printing on desmos
    for (int i = 0; i < n - 2; ++i){
        double l2 = dist({path.p0[i].x, path.p0[i].y}, {path.q0[i].x, path.q0[i].y});
        double l3 = dist({path.q0[i].x, path.q0[i].y}, {path.q1[i].x, path.q1[i].y});
        sol[i] = {
            .a1 = {path.p0[i].x, path.p0[i].y, path.th0[i], 0, 0, path.p0[i].x, path.p0[i].y, path.th0[i]},
            .a2 = {path.p0[i].x, path.p0[i].y, path.th0[i], 0, l2, path.q0[i].x, path.q0[i].y, path.th0[i]},
            .a3 = {path.q0[i].x, path.q0[i].y, path.th0[i], path.k[i], 2 * inv_k * asin(l3 / 2 / inv_k), path.q1[i].x, path.q1[i].y, path.thf[i]},
            .L = l2 + l3
        };
    }

    // build the last straight curve

    point_t last_p0 = {sol[sol.size() - 2].a3.xf, sol[sol.size() - 2].a3.yf};
    double last_th0 = sol[sol.size() - 2].a3.thf;

    point_t last_q0 = {path_poly[path_poly.size() - 1].x(), path_poly[path_poly.size() - 1].y()};
    double last_len = dist(last_p0, last_q0);

    sol.back() = {
        .a1 = {last_p0.x, last_p0.y, last_th0, 0, 0, last_p0.x, last_p0.y, last_th0},
        .a2 = {last_p0.x, last_p0.y, last_th0, 0, last_len, last_q0.x, last_q0.y, last_th0},
        .a3 = {last_q0.x, last_q0.y, last_th0, 0, 0, last_q0.x, last_q0.y, last_th0},
        .L = last_len
    };


    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}