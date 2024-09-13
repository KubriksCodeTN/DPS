/**
 * @file rrtstar.hpp
 * @brief Header of the rrtstar implementation
 */
#pragma once 

#define BOOST_ALLOW_DEPRECATED_HEADERS // xdddddd

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/iterator/function_output_iterator.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <initializer_list>
#include <random>
#include "nanoflann.hpp"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

/**
 * @brief Namespace used by rrtstar
 */
namespace rrtstar{

typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::segment<point_t> segment_t; // maybe not needed?
typedef bg::model::box<point_t> box_t;
typedef box_t value_t;
typedef bgi::rtree<value_t, bgi::rstar<16>> rtree;

/**
 * @brief standard node struct
 */
struct node{    
    std::unordered_set<node*> childrens;
    node* parent;
    point_t p;
    // double d;
    double b;

    node(const std::unordered_set<node*>& otherChildrens, node* otherParent, const point_t& otherPoint, double b = 0)
        : childrens(otherChildrens.begin(), otherChildrens.end()), parent(otherParent), p(otherPoint), b(b){
    }
};

/**
 * @brief point wrapper struct used for the kd_tree's point_cloud
 */
struct point_w{
    double p[2];
    node* link;

    point_w(double x, double y, node* n)
        : link(n){
        p[0] = x;
        p[1] = y;
    }
};

/**
 * @brief point_cloud for the kd_tree
 */
struct point_cloud {
    std::vector<point_w> pts;

    inline size_t kdtree_get_point_count() const {
        return pts.size();
    }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx].p[dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
        return false;
    }
};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, point_cloud>, point_cloud, 2> kd_tree;

double get_distance(node*);
std::vector<node*> rrt(const rtree&, const point_t&, const point_t&, double, double, double, double, double, double);

};
