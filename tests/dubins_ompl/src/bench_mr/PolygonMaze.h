#pragma once

#include "sat.hpp"
#include <memory>
#include <ompl/base/ScopedState.h>

// #include "Environment.h"
// #include "PlannerSettings.h"
#include "SvgPolygonLoader.hpp"

/**
 * Implements a maze consisting of convex shapes as obstacles.
 */
class PolygonMaze {
 public:
  PolygonMaze() = default;

  std::string name() const { return _name; }

  const std::vector<Polygon> &obstacles() const { return _obstacles; }

  static std::shared_ptr<PolygonMaze> loadFromSvg(const std::string &filename) {
    auto maze = std::make_shared<PolygonMaze>();
    maze->_name += " " + filename;
    maze->_obstacles = SvgPolygonLoader::load(filename);
    if (maze->_obstacles.empty()) {
      std::cerr << "Could not find any obstacles in \"" << filename << "\".\n";
      return maze;
    }
    // for (auto &obstacle : maze->_obstacles)
    //   obstacle.scale(global::settings.env.polygon.scaling);
    auto min = maze->_obstacles[0].min();
    auto max = maze->_obstacles[0].max();
    for (const auto &o : maze->_obstacles) {
      const auto new_min = o.min();
      const auto new_max = o.max();
      if (new_min.x < min.x) min.x = new_min.x;
      if (new_max.x > max.x) max.x = new_max.x;
      if (new_min.y < min.y) min.y = new_min.y;
      if (new_max.y > max.y) max.y = new_max.y;
    }
    maze->_bounds.setLow(0, min.x);
    maze->_bounds.setLow(1, min.y);
    maze->_bounds.setHigh(0, max.x);
    maze->_bounds.setHigh(1, max.y);
    OMPL_INFORM(("Loaded polygon maze from \"" + filename + "\".").c_str());
    OMPL_INFORM("\tBounds:  [%.2f %.2f] -- [%.2f %.2f]", min.x, min.y, max.x,
                max.y);
    return maze;
  }

  bool collides(double x, double y) {
    int i = 0;
    for (const auto &poly : _obstacles) {
      if (collision2d::intersect(collision2d::Point<double>{x, y},
                                 (collision2d::Polygon<double>)poly)) {
        #ifdef DEBUG
        OMPL_DEBUG("[%.2f %.2f] collides with polygon %d.", x, y, i);
        #endif
        return true;
      }
      ++i;
    }
    return false;
  }
  bool collides(const Polygon &polygon) {
    for (const auto &poly : _obstacles) {
      if (collision2d::intersect((collision2d::Polygon<double>)polygon,
                                 (collision2d::Polygon<double>)poly)) {
        // std::cerr << "Intersection between polygons " << polygon.min() << " "
        //          << polygon.max() << " and " << poly.min() << " " <<
        //          poly.max()
        //          << std::endl;
        return true;
      }
    }
    return false;
  }

  /*void to_json(nlohmann::json &j) override {
    j["type"] = "polygon";
    j["obstacles"] = obstacles();
    j["start"] = {start().x, start().y, startTheta()};
    j["goal"] = {goal().x, goal().y, goalTheta()};
    j["width"] = width();
    j["height"] = height();
    j["min_x"] = _bounds.low[0];
    j["min_y"] = _bounds.low[1];
    j["max_x"] = _bounds.high[0];
    j["max_y"] = _bounds.high[1];
    j["name"] = name();
  }*/

  double unit() const { return .2; }

  std::string _name{"polygon_maze"};
  std::vector<Polygon> _obstacles;
  ob::RealVectorBounds _bounds{2};
};
