/**
 * @file rrt_dubins.cpp
 * @brief test for rrt_dubins
 */
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>

#include "bench_mr/PolygonMaze.h"

#include <iostream>
#include <fstream>

struct state_t {
    double x, y, th;
};

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

// bottom-left and top-right vertices
struct Obstacle {
    double x_bl, y_bl, x_tr, y_tr;
};

std::vector<Obstacle> obstacles;

/**
 * @brief checks whether a state is valid or not
 */
bool isStateValid(const ob::SpaceInformation *si, const ob::State *state) {
    const auto *dubinsState = state->as<ob::SE2StateSpace::StateType>();
    double x = dubinsState->getX();
    double y = dubinsState->getY();
    double yaw = dubinsState->getYaw();

    // Check if the state is within the bounds
    if (!si->satisfiesBounds(state)) {
        return false;
    }

    // Check for collision with each obstacle
    for (const auto &obs : obstacles) {
        if(obs.x_bl < x && x < obs.x_tr && obs.y_bl < y && y < obs.y_tr) {
            // std::cerr << obs.x_bl << " " << obs.x_tr << " " << obs.y_bl << " " << obs.y_tr << "\n" ;
            return false;
        }
    }

    return true;
}

/**
 * @brief attempt to find a feasible solution using OMPL's rrt_dubins
 */
void plan(double s_x, double s_y, double s_th, double endx, double endy, double endth, const std::shared_ptr<PolygonMaze> p, const ob::StateSpacePtr space, double& len, double& time, double max_time)
{
    ob::ScopedState<> start(space), goal(space);
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(0);
    // bounds.setHigh(0, endx + 10);
    // bounds.setHigh(1, endy + 10);

    space->as<ob::DubinsStateSpace>()->setBounds(p->_bounds);

    len = 0;
    time = 0;    

    // define a simple setup class
    og::SimpleSetup ss(space);
    ss.getSpaceInformation()->setStateValidityChecker([&ss](const ob::State *state) {
        return isStateValid(ss.getSpaceInformation().get(), state);
    });
    // Setup a dubins motion validator that uses the state validity checker
    ss.getSpaceInformation()->setMotionValidator(std::make_shared<ob::DubinsMotionValidator>(ss.getSpaceInformation()));
    ss.getSpaceInformation()->setStateValidityCheckingResolution(1.e-4); // resolution when cecking path collision


    // change the planner
    auto rrt_star(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    ss.setPlanner(rrt_star); 

    // auto planner = ss.getPlanner();
    // std::string name = planner->getName();

    // set start and goal
    start[0] = s_x;
    start[1] = s_y;
    start[2] = s_th;

    goal[0] = endx;
    goal[1] = endy;
    goal[2] = endth;

    //std::cerr << start << " --- " << goal << "\n";

    ss.setStartAndGoalStates(start, goal);

    ob::PlannerStatus solved = ss.solve(max_time);

    if (solved)
    {
        og::PathGeometric &dubins_path = ss.getSolutionPath();
        
        ss.simplifySolution(); 

        //dubins_path.interpolate(500);
        auto path = dubins_path.getStates();
        double last_x = path.back()->as<ob::SE2StateSpace::StateType>()->getX();
        double last_y = path.back()->as<ob::SE2StateSpace::StateType>()->getY();

        if(std::fabs(last_x - endx) > .1 || std::fabs(last_y - endy) > .1 ){
            std::cerr << "SOLUTION NOT FOUND\n" ;

            exit(18);
        }

        len = dubins_path.length();
        time = (ss.getLastPlanComputationTime() + ss.getLastSimplificationTime()) * 1.e6; // from s to us
        //std::cerr << ss.getLastPlanComputationTime() * 1.e6 << " " << ss.getLastSimplificationTime() * 1.e6 << "\n";


        dubins_path.printAsMatrix(std::cout);    
    }
    else
        std::cout << "No solution found" << std::endl;

}

int main(int argc, char* argv[])
{

    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    double turn_r = .5;

    if(argc != 9){
        std::cerr << "Usage: rrt start_x start_y start_th final_x final_y final_th svg_path max_time";
        return 1;
    }

    auto p = PolygonMaze::loadFromSvg(argv[7]);
    

    for(auto& obs : p->obstacles()){
        auto [x_bl, y_bl] = obs.min();
        auto [x_tr, y_tr] = obs.max();

        obstacles.push_back({x_bl, y_bl, x_tr, y_tr});
    }

    for(const auto& obs : obstacles)
        std::cout << std::fixed << "\\operatorname{polygon}((" << obs.x_bl << "," << obs.y_bl << "),(" 
            << obs.x_tr << "," << obs.y_bl << "),("
            << obs.x_tr << "," << obs.y_tr << "),("
            << obs.x_bl << "," << obs.y_tr << "))\n";

    try
    {
        ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(turn_r));

        double time, len;
        plan(std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6]), p, space, len, time, std::atof(argv[8]));

        std::cout << std::fixed << "time: " << time << " us | len: " << len << "\n";
        
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
