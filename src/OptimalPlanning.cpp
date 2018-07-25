//Optimal Motion Planning, OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For boost::make_shared
#include <boost/make_shared.hpp>

#include <fstream>


namespace ob = ompl::base;
namespace og = ompl::geometric;

enum optimalPlanner{
	PLANNER_BITSTAR,
	PLANNER_CFOREST,
	PLANNER_FMTSTAR,
	PLANNER_INF_RRTSTAR,
	PLANNER_PRMSTAR,
	PLANNER_RRTSTAR
};

enum planningObjective{
	OBJECTIVE_PATHCLEARANCE,
	OBJECTIVE_PATHLENGTH,
	OBJECTIVE_THRESHOLDPATHLENGTH,
	OBJECTIVE_WEIGHTEDCOMBO
};

//parse the command-line arguments
bool argParse(int argc, char** argv, double *runTime, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr);

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
    }
};

//pre define the get objective function
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

//ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

//ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

//ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

//ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr allocateObjective(ob::SpaceInformationPtr si, planningObjective objectiveType){
	switch(objectiveType){
		/*case OBJECTIVE_PATHCLEARANCE:
			return getClearanceObjective(si);
			break;*/
		case OBJECTIVE_PATHLENGTH:
			return getPathLengthObjective(si);
			break;
		case OBJECTIVE_THRESHOLDPATHLENGTH:
			return getThresholdPathLengthObj(si);
			break;
		/*case OBJECTIVE_WEIGHTEDCOMBO:
			return getBalancedObjective1(si);
			break;*/
		default:
			OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
			return ob::OptimizationObjectivePtr(); //return an empty object??
			break;
	}
}

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        case PLANNER_BITSTAR:
        {
            return boost::make_shared<og::BITstar>(si);
            break;
        }
        case PLANNER_CFOREST:
        {
            return boost::make_shared<og::CForest>(si);
            break;
        }
        case PLANNER_FMTSTAR:
        {
            return boost::make_shared<og::FMT>(si);
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            return boost::make_shared<og::InformedRRTstar>(si);
            break;
        }
        case PLANNER_PRMSTAR:
        {
            return boost::make_shared<og::PRMstar>(si);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return boost::make_shared<og::RRTstar>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

void plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, std::string outputFile){
	//construct the robot state
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

	// Set the bounds of space to be in [0,1].
	space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

	//construct a space information instance for the state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

	//set the object used to check which states in the space are valid
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

	si->setup();

	// Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    //create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    //set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    //create the optimization objective
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);

    if (solved)
    {
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        if (!outputFile.empty())
        {
            std::ofstream outFile(outputFile.c_str());
            boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                printAsMatrix(outFile);
            outFile.close();
            //printf("%s",outputFile.c_str());
            //pdef->getSolutionPath()->printAsMatrix(std::cout);
        }else{
            printf("output file empty!\n");
        }
    }
    else
        std::cout << "No solution found." << std::endl;

}

int main(int argc, char** argv){
	double runTime;
	optimalPlanner plannerType;
	planningObjective objectiveType;
	std::string outputFile;


	if(argParse(argc, argv, &runTime, &plannerType, &objectiveType, &outputFile)){
		//plan
		plan(runTime, plannerType, objectiveType, outputFile);

		//Return with success
		return 0;
	}
	else{
		return -1;
	}
}

//get objective function
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}


bool argParse(int argc, char** argv, double* runTimePtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr, std::string *outputFilePtr)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BITstar, CForest, FMTstar, InformedRRTstar, PRMstar, and RRTstar.") //Alphabetical order
        ("objective,o", bpo::value<std::string>()->default_value("PathLength"), "(Optional) Specify the optimization objective, defaults to PathLength if not given. Valid options are PathClearance, PathLength, ThresholdPathLength, and WeightedLengthAndClearanceCombo.") //Alphabetical order
        ("file,f", bpo::value<std::string>()->default_value("path.txt"), "(Optional) Specify an output path for the found solution path.")
        ("info,i", bpo::value<unsigned int>()->default_value(0u), "(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to WARN.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }

    // Set the log-level
    unsigned int logLevel = vm["info"].as<unsigned int>();

    // Switch to setting the log level:
    if (logLevel == 0u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }
    else if (logLevel == 1u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    }
    else if (logLevel == 2u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    }
    else
    {
        std::cout << "Invalid log-level integer." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the runtime as a double
    *runTimePtr = vm["runtime"].as<double>();

    // Sanity check
    if (*runTimePtr <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("BITstar", plannerStr))
    {
        *plannerPtr = PLANNER_BITSTAR;
    }
    else if (boost::iequals("CForest", plannerStr))
    {
        *plannerPtr = PLANNER_CFOREST;
    }
    else if (boost::iequals("FMTstar", plannerStr))
    {
        *plannerPtr = PLANNER_FMTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("PRMstar", plannerStr))
    {
        *plannerPtr = PLANNER_PRMSTAR;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        *plannerPtr = PLANNER_RRTSTAR;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified objective as a string
    std::string objectiveStr = vm["objective"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("PathClearance", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHCLEARANCE;
    }
    else if (boost::iequals("PathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHLENGTH;
    }
    else if (boost::iequals("ThresholdPathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_THRESHOLDPATHLENGTH;
    }
    else if (boost::iequals("WeightedLengthAndClearanceCombo", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_WEIGHTEDCOMBO;
    }
    else
    {
        std::cout << "Invalid objective string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the output file string and store it in the return pointer
    *outputFilePtr = vm["file"].as<std::string>();

    // Looks like we parsed the arguments successfully
    return true;
}
/// @endcond
