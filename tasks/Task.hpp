/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MOTION_PLANNING_TASK_TASK_HPP
#define MOTION_PLANNING_TASK_TASK_HPP

#include "motion_planning/TaskBase.hpp"
#include "motion_planning/MotionPlanning.hpp"

namespace motion_planning{
    
    class Task : public TaskBase
    {
		friend class TaskBase;
    	protected:
		    MotionPlanning_lib::MotionPlanning *motionPlanner;

			// Properties
			char* scriptFile;
			char* pathVariable;
			char* headingVariable;
			char* jointsVariable;
			char* assignmentVariable;
			int numJoints;
			double *aRoverPath;
			double *aRoverHeading;
			double *aJoints;
			int *aAssignment;

			double xm, ym, xr, yr, initHeading, resolution, size;
			char* mapDirectory;

			// Output ports
			int sizePath;
			std::vector<base::Waypoint> roverPath;
            ArmProfile armProfile;
			std::vector<int> assignment;

			// Local variables
			PyObject *pyModule;

			std::vector<double> joints;
			
		

		public:

		    Task(std::string const& name = "motion_planning::Task");
		    Task(std::string const& name, RTT::ExecutionEngine* engine);

		~Task();


		    bool configureHook();
		    bool startHook();
		    void updateHook();
		    void errorHook();
		    void stopHook();
		    void cleanupHook();
    };
}

#endif

