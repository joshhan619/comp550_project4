///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <cmath>

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const double theta = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(0)->value;
        const double omega = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
        projection(0) = theta;
        projection(1) = omega;
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // Fill in the ODE for the pendulum's dynamics
    qdot.resize(q.size(), 0);
    double tau = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    qdot[0] = q[1];
    qdot[1] = -9.81*cos(q[0])+tau;
}

void postPropagate(const ob::State */*state*/, const oc::Control */*control*/, const double /*duration*/, ob::State *result) {
    ob::SO2StateSpace SO2;
    ob::SO2StateSpace::StateType *s = result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    SO2.enforceBounds(s);
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    // Create pendulum's state space
    auto space (std::make_shared<ob::CompoundStateSpace>());
    space->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 1);
    space->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1);

    ob::RealVectorBounds bounds(1);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ob::RealVectorStateSpace>(1)->setBounds(bounds);

    // Create pendulum's control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 1));

    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);

    // Create SimpleSetup and add state space and control space
    auto ss (std::make_shared<oc::SimpleSetup>(cspace));

    // Add validity checker
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si](const ob::State *state)
    { 
        return si->satisfiesBounds(state);
    });

    // Add state propagator
    oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (ss->getSpaceInformation(), &pendulumODE));
    si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    ob::ScopedState<ob::CompoundStateSpace> start(space);
    start[0] = -M_PI/2;
    start[1] = 0;

    ob::ScopedState<ob::CompoundStateSpace> goal(space);
    goal[0] = M_PI/2;
    goal[1] = 0;

    ss->setStartAndGoalStates(start, goal, 0.1);

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    ob::PlannerPtr planner;
    if (choice == 1) {
        // Use RRT
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    } else if (choice == 2) {
        // Use KPIECE
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        // Define a projection for the state space
        ob::StateSpace *space = ss->getStateSpace().get();
        space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PendulumProjection(space)));
    } else if (choice == 3) {
        // Use RG-RRT
        planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
    }
    ss->setPlanner(planner);
    ss->setup();
    ob::PlannerStatus solved = ss->solve(60);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().print(std::cout);
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr & ss )
{
    // TODO: Do some benchmarking for the pendulum
    ompl::tools::Benchmark b(*ss, "Project 4b Pendulum");

    ob::StateSpace *space = ss->getStateSpace().get();
    space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PendulumProjection(space)));
    // Add benchmark planners
    b.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(ss->getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation())));
    
    // Create a benchmark request
    ompl::tools::Benchmark::Request req;
    req.maxTime = 60.0;
    req.maxMem = 10000.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile("pendulum_benchmarking.log");
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
