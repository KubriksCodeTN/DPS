#include "planner.hpp"

int main(int argc, char** argv){

    auto node = Planner();

    uint64_t n_points;
    double x,y;
    VisiLibity::Polyline shortest_path;
    
    std::cin >> n_points;
    
    while(n_points--){
        fscanf(stdin, "%lf,%lf", &x, &y);
        shortest_path.push_back({x, y});
    }

    double time, len;

    node.test(shortest_path, time, len);
  
}