///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd> projection ) const override
    {
        // TODO: Your projection for the car
        const auto *compoundState = state->as<ob::CompoundStateSpace::StateType>();
        const auto *se2State = compoundState->as<ob::SE2StateSpace::StateType>(0);

        projection[0] = se2State->getX();
        projection[1] = se2State->getY();
    }
};

void carODE(const ompl::control::ODESolver::StateType &  q , const ompl::control::Control * control ,
            ompl::control::ODESolver::StateType &  qdot )
{
    const double w= control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    const double vdot= control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    qdot.resize(q.size(), 0); //make sure same size as statetype q
    qdot[0]=q[3]*cos(q[2]);
    qdot[1]=q[3]*sin(q[2]);
    qdot[2]=w;
    qdot[3]=vdot;
    
}

void makeStreet(std::vector<Rectangle> &  obstacles )
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obs1;
    obs1.x =-6;
    obs1.y=-10;
    obs1.width=10;
    obs1.height=4;
    obstacles.push_back(obs1);

    Rectangle obs2;
    obs2.x =-6;
    obs2.y=6;
    obs2.width=10;
    obs2.height=2;
    obstacles.push_back(obs2);
    
    Rectangle obs3;
    obs3.x =-6;
    obs3.y=-4;
    obs3.width=5;
    obs3.height=8;
    obstacles.push_back(obs3);
    
    Rectangle obs4;
    obs4.x =2;
    obs4.y=-4;
    obs4.width=4;
    obs4.height=8;
    obstacles.push_back(obs4);
    

}
void postPropagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) {
    
    const auto *compoundState = state->as<ob::CompoundStateSpace::StateType>();
    const auto *se2State = compoundState->as<ob::SE2StateSpace::StateType>(0);
    const auto *velocityState =compoundState->as<ob::RealVectorStateSpace::StateType>(1);;

    double x = se2State->getX();
    double y = se2State->getY();
    double theta = se2State->getYaw();  
    double v = velocityState->values[0]; 

    const double *controlValues = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double w = controlValues[0];  
    double v_dot = controlValues[1];  
    
    double t = 0;
    while ( t < duration)  // Use small time steps for better accuracy
    {
        v += v_dot * 0.01;
        x += v * cos(theta) * 0.01;
        y += v * sin(theta) * 0.01;
        theta += w * 0.01;
        t+=0.01;
        theta = std::fmod(theta + M_PI, 2 * M_PI) - M_PI;

    }
    // Set the result state
    auto *result_CompoundState = result->as<ob::CompoundStateSpace::StateType>();
    auto *result_se2State = result_CompoundState->as<ob::SE2StateSpace::StateType>(0);
    auto *result_velocityState = result_CompoundState->as<ob::RealVectorStateSpace::StateType>(1);

    result_se2State->setX(x);
    result_se2State->setY(y);
    result_se2State->setYaw(theta);
    result_velocityState->values[0] = v;
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &  obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    // Create a compound state space
    auto space = std::make_shared<ob::CompoundStateSpace>();

    auto se2_space = std::make_shared<ob::SE2StateSpace>();
    auto velocity_space = std::make_shared<ob::RealVectorStateSpace>(1);
    space->addSubspace(se2_space, 1.0);
    space->addSubspace(velocity_space, 1.0);
    

    ob::RealVectorBounds se2_bounds(2);  
    se2_bounds.setLow(0, -6);  
    se2_bounds.setHigh(0, 6);  //x
    se2_bounds.setLow(1, -10);  
    se2_bounds.setHigh(1, 10);  //y
    se2_bounds.setLow(2, -M_PI);  //  yaw
    se2_bounds.setHigh(2, M_PI);  //  yaw
    se2_space->setBounds(se2_bounds);

    ob::RealVectorBounds velocity_bounds(1);  // Velocity 
    velocity_bounds.setLow(-5);  
    velocity_bounds.setHigh(5);  
    velocity_space->setBounds(velocity_bounds);

     // create a control space
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 2);
  
     // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
  
    cspace->setBounds(cbounds);
  
     // define a simple setup class
    auto ss (std::make_shared<oc::SimpleSetup>(cspace));
  
      // Add validity checker
    oc::SpaceInformation *si = ss->getSpaceInformation().get();

    ss->setStateValidityChecker([si, &obstacles](const ob::State *state) 
    { 
        const auto *compoundState = state->as<ob::CompoundStateSpace::StateType>();
        const auto *se2State = compoundState->as<ob::SE2StateSpace::StateType>(0);

        double x = se2State->getX();
        double y = se2State->getY();
        // Check if the state satisfies bounds and is valid regarding obstacles
        return si->satisfiesBounds(state) && isValidPoint(x,y, obstacles);
       // return si->satisfiesBounds(state) ;
    });


    // Add state propagator
    oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (ss->getSpaceInformation(), &carODE));
    si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    ob::ScopedState<ob::CompoundStateSpace> start(space);
    start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;  // Set initial velocity
    auto *se2_start = start->as<ob::SE2StateSpace::StateType>(0); 
    se2_start->setX(-5);
    se2_start->setY(-5);
    se2_start->setYaw(0);

    ob::ScopedState<ob::CompoundStateSpace> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;  // Goal velocity
    auto *se2_goal = goal->as<ob::SE2StateSpace::StateType>(0);
    se2_goal->setX(4);
    se2_goal->setY(5);
    se2_goal->setYaw(0);

    ss->setStartAndGoalStates(start, goal,1);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr & ss , int choice )
{
    // TODO: Do some motion planning for the car
    ob::PlannerPtr planner;
    if (choice == 1) {
        // Use RRT
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    } else if (choice == 2) {
        // Use KPIECE
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        // Define a projection for the state space
        ob::StateSpace *space = ss->getStateSpace().get();
        space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarProjection(space)));
    } else if (choice == 3) {
        // Use RG-RRT
        planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
    }
    ss->setPlanner(planner);
    ss->setup();
    ob::PlannerStatus solved = ss->solve(120);

    if (solved && ss->getSolutionPath().getStateCount() > 0) {
        ss->getSolutionPath().print(std::cout);
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
