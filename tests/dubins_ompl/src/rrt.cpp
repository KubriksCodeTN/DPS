/**
 * @file rrt.cpp
 * @brief test for rrt*
 */
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/DiscreteMotionValidator.h>
//#include <omplapp/geometry/detail/FCLContinuousMotionValidator.h>

#include "bench_mr/PolygonMaze.h"
#include "planner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

ob::State* start_state_global;
double min_dist_global;
std::shared_ptr<og::SimpleSetup> ss_global;

// bottom-left and top-right vertices
struct Obstacle {
    double x_bl, y_bl, x_tr, y_tr;
};

std::vector<Obstacle> obstacles;
const double r = 0.5;

/**
 * @brief checks whether a state is valid or not
 */
bool isStateValid(const ob::SpaceInformation *si, const ob::State *state) {
    
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX();
    double y = s->getY();

    // static int i = 0;
    // std::cerr << "\n\nstate " << i++ << ": " << x << " " << y << " | ";

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
 * @brief custom sampler class for rrtstar 
 */
class CustomStateSampler : public ob::StateSampler{
public:
    CustomStateSampler(const ob::StateSpace *space, double minDistance, ob::State* start)
        : ob::StateSampler(space), minDistance_(minDistance), previousState_(nullptr)
    {
        previousState_ = space_->allocState();
        tmp_state_ = space_->allocState();

        space_->copyState(previousState_, start);
        previousState_->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI_2);
    }

    ~CustomStateSampler()
    {
        if (previousState_)
            space_->freeState(previousState_);
    }

    void sampleUniform(ob::State *state) override
    {

        double th = rng.uniformReal(-M_PI * .6666666, M_PI * .6666666);
        double dist = minDistance_; //uniformReal(minDistance_, 4 * minDistance_); //space_->as<ob::SE2StateSpace>()->getBounds().high[1]);
        auto* tmp = tmp_state_->as<ob::SE2StateSpace::StateType>();
        auto* prev = previousState_->as<ob::SE2StateSpace::StateType>();

        double sn, cs;
        sincos(tmp->getYaw(), &sn, &cs);
        double true_dist = dist + rng.uniformReal(0, 45);

        tmp->setYaw(prev->getYaw() + th);
        tmp->setX(prev->getX() + true_dist * cs);
        tmp->setY(prev->getY() + true_dist * sn);

        // fprintf(stderr, "%lf %lf - %lf %lf - %lf\n", 
        //     prev->getX(),
        //     prev->getY(),
        //     tmp->getX(),
        //     tmp->getY(),
        //     dist_sq
        // );

        if(!isStateValid(ss_global->getSpaceInformation().get(), tmp))
            return;
        
        //printf("VALID\n");

        space_->copyState(previousState_, tmp);
        space_->copyState(state, tmp);
    }

    void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override
    {
        printf("yo I shouldn't be here\n");
        sampleUniform(state); // Not implemented for simplicity
    }

    void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override
    {
        sampleUniform(state); // Not implemented for simplicity
    }

private:
    double minDistance_;
    ob::State *previousState_, *tmp_state_;
    ompl::RNG rng;
};

/**
 * @brief allocates the custom sampler
 */
ob::StateSamplerPtr alloc2(const ob::StateSpace *sp){
    return std::make_shared<CustomStateSampler>(sp, min_dist_global, start_state_global);
}

/**
 * @brief attempts to find a feasible solution using rrtstar and the custom sampler
 */
void planWithCustomSampler(double s_x, double s_y, double f_x, double f_y, const std::shared_ptr<PolygonMaze> p, double& len, double& time, double max_time, double& th0, double& thf)
{
    auto space(std::make_shared<ob::SE2StateSpace>());

    space->setBounds(p->_bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    ss.getSpaceInformation()->setStateValidityChecker([&](const ob::State *state) {
        return isStateValid(ss.getSpaceInformation().get(), state);
    });

    ss.getSpaceInformation()->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(ss.getSpaceInformation()));
    ss.getSpaceInformation()->setStateValidityCheckingResolution(1.e-4); // resolution when cecking path collision
    //std::cerr << ss.getSpaceInformation()->getStateValidityCheckingResolution()<<"\n\n";


    ob::ScopedState<> start(space), goal(space);

    start[0] = s_x;
    start[1] = s_y;
    //start[2] = M_PI_2;
    goal[0] = f_x;
    goal[1] = f_y;
    //goal[2] = M_PI_2;

    ss.setStartAndGoalStates(start, goal);

    // Set up the planner
    auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Use the custom sampler allocator
    start_state_global = space->allocState();
    min_dist_global = 1.72;
    ss_global = std::make_shared<og::SimpleSetup>(ss);
    auto* dub_start = start_state_global->as<ob::SE2StateSpace::StateType>();
    dub_start->setX(start[0]);
    dub_start->setY(start[1]);
    //space->setStateSamplerAllocator(alloc2);

    ob::PlannerStatus solved = ss.solve(max_time);

    if (solved)
    {
        og::PathGeometric &path = ss.getSolutionPath();
        
        ss.simplifySolution();

        //path.interpolate(500);

        //len = path.length();
        // std::cerr << ss.getLastPlanComputationTime() * 1.e6 << " " << ss.getLastSimplificationTime() * 1.e6 << "\n";
        time = (ss.getLastPlanComputationTime() + ss.getLastSimplificationTime()) * 1.e6; // from s to us

        VisiLibity::Polyline p_path;

        for(auto s : path.getStates()){
            p_path.push_back({
                s->as<ob::SE2StateSpace::StateType>()->getX(),
                s->as<ob::SE2StateSpace::StateType>()->getY()
            });
        }   

        path.printAsMatrix(std::cout);

        if(std::fabs(p_path[p_path.size() - 1].x() - f_x) > .1 || std::fabs(p_path[p_path.size() - 1].y() - f_y) > .1 ){
            std::cerr << "SOLUTION NOT FOUND\n" ;

            exit(18);
        }


        int n = p_path.size() - 3;

        for(int i = 0; i < n; ++i){
            double p0x = p_path[i].x();
            double p0y = p_path[i].y();
            double p1x = p_path[i + 1].x();
            double p1y = p_path[i + 1].y();
            double p2x = p_path[i + 2].x();
            double p2y = p_path[i + 2].y();

            double d0 = sqrt((p1x - p0x)*(p1x - p0x) + (p1y - p0y)*(p1y - p0y));
            double d1 = sqrt((p2x - p1x)*(p2x - p1x) + (p2y - p1y)*(p2y - p1y));

            double cross0 = p0x * p1y - p0y * p1x;
            double dot0 = p0x * p1x + p0y * p1y;
            double tan0_2 = cross0 / (dot0 + d0);

            double cross1 = p1x * p2y - p1y * p2x;
            double dot1 = p1x * p2x + p1y * p2y;
            double tan1_2 = cross1 / (dot1 + d1);
            
    
            if(!i)
                if(d0 < r * tan0_2){
                    //std::cout << i << ": " << th1 << " " << th2 << " " << d << " " << 0.5 * (tan(th1 / 2)) << "\n";
                    std::cout << "1 Solution is found but it doesn't fulfill the requirements" << std::endl;
                    exit(18);
                }
                else continue;
        
            if(i == n - 1)
                if(d0 < r * tan1_2){
                    //std::cout << i << ": " << th1 << " " << th2 << " " << d << " " << 0.5 * (tan(th2 / 2)) << "\n";
                    std::cout << "2 Solution is found but it doesn't fulfill the requirements" << std::endl;
                    exit(18);
                }
                else continue;

            if(d0 < r * (tan0_2 + tan1_2)){
                //std::cout << i << ": " << th1 << " " << th2 << " " << d << " " << 0.5 * (tan(th1 / 2) + tan(th2 / 2)) << "\n";
                std::cout << "3 Solution is found but it doesn't fulfill the requirements" << std::endl;
                exit(18);
            }
        }

        th0 = atan2(p_path[1].y() - p_path[0].y(), p_path[1].x() - p_path[0].x());
        thf = atan2(p_path[p_path.size() - 2].y() - p_path[p_path.size() - 1].y(), p_path[p_path.size() - 2].x() - p_path[p_path.size() - 1].x());

        auto planner = Planner();
        double t = 0;
        planner.dps(p_path, t, len);

        time += t;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        exit(18);
    }
}

int main(int argc, char** argv)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    if(argc != 7){
        std::cerr << "Usage: rrt start_x start_y final_x final_y svg_path max_time";
        return 1;
    }

    auto p = PolygonMaze::loadFromSvg(argv[5]);

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
    double len, time, th0, thf;

    planWithCustomSampler(std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]), p, len, time, std::atof(argv[6]), th0, thf);

    std::cout << std::fixed << "time: " << time << " us | len: " << len << " | th0: " << th0 << " | thf: " << thf << "\n";
    return 0;
}