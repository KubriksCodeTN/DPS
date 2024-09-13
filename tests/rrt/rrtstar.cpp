/**
 * @file rrtstar.cpp
 * @brief implementation of rrt* algorithm
 */
#include <chrono>
#include <iostream>
#include "rrtstar.hpp"

// constexpr uint64_t n_iter = 250000; // ???

// yes, this is garbage
using namespace rrtstar;

/**
 * @brief radius of the robot
 */
const double r = .5; // shelfino's radius

/**
 * @brief prints the tree starting from the root n
 */
void print_tree(node* n){
    std::cout << "(" << n->p.get<0>() << "," << n->p.get<1>() << ")\n";
    for (const auto x : n->childrens)
        print_tree(x);
}

/**
 * @brief Euclidean distance
 */
inline double L2(const point_t& a, const point_t& b){
    return (b.get<0>() - a.get<0>()) * (b.get<0>() - a.get<0>()) + (b.get<1>() - a.get<1>()) * (b.get<1>() - a.get<1>());
}

/**
 * @brief changes the parent of a given node
 * 
 * @param parent old parent
 * @param son node to change parent to
 * @param newParent new parent 
 */
inline void change_edge(node* parent, node* son,node* newParent) {
    parent->childrens.erase(son);
    son->parent = newParent;
    newParent->childrens.insert(son);
}

/**
 * @brief given a point p gets the nearest node in the tree
 * 
 * @param index a kd_tree with all nodes in the tree
 * @param cloud point_cloud auxiliary structure for node indexing
 * @param p the point p
 * @return the node with the nearest point
 */
node* get_nearest(kd_tree* index, const point_cloud& cloud, const point_t& p){
    constexpr size_t n = 1;
    size_t res_i;
    double l2_sqr;
    double query[2] = {p.get<0>(), p.get<1>()};
    nanoflann::KNNResultSet<double> result_set(n);
    result_set.init(&res_i, &l2_sqr);
    index->findNeighbors(result_set, query, nanoflann::SearchParameters());
    return cloud.pts[res_i].link;
}

/**
 * @brief given a node p, find the nodes in the tree that are at a distance of no more than step from it.
 * 
 * @param index a kd_tree with all nodes in the tree
 * @param cloud point_cloud auxiliary structure for node indexing 
 * @param p the point p
 * @param step maximum distance from the node
 * @param gamma [unused]
 * @param N number of nodes in the tree
 * @param [out] result the set of found nodes
 */
void get_near(kd_tree* index, const point_cloud& cloud, const point_t& p, const double step, 
    [[maybe_unused]] const double gamma, const double N, std::vector<node*>& result){
    double distance = step; // std::min(step, gamma * std::sqrt(log((double)N) / (double)(N))); // <--
    distance *= distance;

    std::vector<nanoflann::ResultItem<size_t, double>> res_matches;
    double query_pt[2] = {p.get<0>(), p.get<1>()};
    nanoflann::RadiusResultSet<double, size_t> result_set(distance, res_matches);
    index->findNeighbors(result_set, query_pt, nanoflann::SearchParameters());

    for (const auto& [index, _] : res_matches) {
        result.push_back(cloud.pts[index].link);
    }
}

/**
 * @brief inserts a node and an edge in the tree
 */
node* insert_v_e(kd_tree* index, point_cloud& cloud, [[maybe_unused]] /* const */ auto& used, node* n, const point_t& p){
    node* new_n = new node(std::unordered_set<node*>(), n, p);
    n->childrens.insert(new_n);
    cloud.pts.emplace_back(p.get<0>(), p.get<1>(), new_n);
    index->addPoints(cloud.pts.size() - 1, cloud.pts.size() - 1);
    // ++N; // bruh?
    return new_n;
}

/**
 * @brief steer function
 */
inline point_t steer(const point_t& a, const point_t& b, double step){
    double norm = std::sqrt(L2(a, b));
    if (norm <= step)
        return b;
    double ax = a.get<0>(), ay = a.get<1>(), bx = b.get<0>(), by = b.get<1>();
    return point_t{ax + (bx - ax) / norm * step, ay + (by - ay) / norm * step};
}

/**
 * @brief given a node x and a point y checks wheter the segment formed by them is collision free in the current environment
 * 
 * @param rt the rtree of the current environment
 * @param x the node x
 * @param y the point y
 * @return true if the segment is collision free else false
 */
bool obstacle_free(const rtree& rt, node* x, const point_t& y){
    segment_t s(x->p, y);
    return !rt.query(bgi::intersects(s), boost::make_function_output_iterator([](const auto& v){}));
}

/**
 * @brief given a node p gets the total length to go from p to the root passing through the intermediate points
 */
double rrtstar::get_distance(node* p){
    double res = 0;
    const node* prev = p;
    const node* cur = prev->parent;
    while (cur != nullptr) {
        res += std::sqrt(L2(prev->p, cur->p));
        prev = cur;
        cur = cur->parent;
    }
    return res;
}

/**
 * @brief rrt* algorithm
 * 
 * @param map the rtree of the current environment
 * @param start starting point
 * @param end ending point
 * @param h height of the map
 * @param l length of the map
 * @param e goal tolerance
 * @param stepsz
 * @param gamma
 * @param s timeout in seconds
 * @return array of found points that are within e from the goal
 */
std::vector<node*> rrtstar::rrt(const rtree& map, const point_t& start, const point_t& end, double h, double l, double e,
    double stepsz, double gamma, double s){
    static std::mt19937 mt(std::random_device{}());

    // TODO maybe remove
    std::uniform_real_distribution<double> hd(0, h);
    std::uniform_real_distribution<double> ld(0, l);

    uint32_t Ne = 0;
    uint32_t N = 1;
    double eq = e * e;
    
    // node* end_node = nullptr;
    std::vector<node*> end_nodes;

    node* root = new node(std::unordered_set<node*>(), nullptr, start);
    point_cloud cloud;
    cloud.pts.emplace_back(start.get<0>(), start.get<1>(), root);
    kd_tree* index = new kd_tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    std::unordered_map<double, std::unordered_set<double>> used;

    point_t x_rand;

    auto it = std::chrono::high_resolution_clock::now();
    auto et = std::chrono::high_resolution_clock::now();
    bool stop = true;

    for (; stop;){
        x_rand = {hd(mt), ld(mt)}; // x
        node* x_near = get_nearest(index, cloud, x_rand); // y
        point_t x_new = steer(x_near->p, x_rand, stepsz); // z

        if (L2(x_near->p, x_new) < .01)
            continue;

        if (std::chrono::duration_cast<std::chrono::milliseconds>(et - it).count() > s * 1e3){
            x_new = end;
            stop = false;
        }

        // TODO fix
        if (obstacle_free(map, x_near, x_new)){
            node* x_min = x_near;
            std::vector<node*> x_nears;
            get_near(index, cloud, x_new, stepsz, gamma, N, x_nears);

            for (const auto& x : x_nears){
                // TODO cache the values omggg
                if (L2(x->p, x_new) >= .01 && obstacle_free(map, x, x_new))
                    if (get_distance(x) + std::sqrt(L2(x->p, x_new)) <
                        get_distance(x_min) + std::sqrt(L2(x_min->p, x_new)))
                        x_min = x;
            }

            node* new_node = insert_v_e(index, cloud, used, x_min, x_new);
            // TODO what
            if (new_node){
                ++N;
                ++Ne;
                for (const auto& x : x_nears){
                    if (x == x_min) continue;

                    // TODO what is this L2 check man
                    if (L2(x->p, x_new) >= 0.01 && obstacle_free(map, x, x_new) &&
                        get_distance(x) > get_distance(new_node) + std::sqrt(L2(x->p, new_node->p))){
                        auto x_p = x->parent;
                        change_edge(x_p, x, new_node);
                    }
                }
            }
            if (L2(new_node->p, end) <= eq){
                end_nodes.push_back(new_node);
            }
        }
        et = std::chrono::high_resolution_clock::now();
    }
    // std::cerr << end_nodes.size() << '\n';
    // print_tree(root);
    return end_nodes;
}
