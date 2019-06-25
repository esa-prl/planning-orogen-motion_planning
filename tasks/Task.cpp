/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace motion_planning;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
	

	std::string string_aux;	

	string_aux = _scriptFile.get();
	scriptFile = new char[string_aux.length() + 1];
	strcpy(scriptFile, string_aux.c_str());

	string_aux =  _pathVariable.get();	
	pathVariable = new char[string_aux.length() + 1];
	strcpy(pathVariable, string_aux.c_str());

	string_aux =  _headingVariable.get();
	headingVariable = new char[string_aux.length() + 1];
	strcpy(headingVariable, string_aux.c_str());

	string_aux =  _jointsVariable.get();
	jointsVariable = new char[string_aux.length() + 1];
	strcpy(jointsVariable, string_aux.c_str());

	string_aux =  _assignmentVariable.get();
	assignmentVariable = new char[string_aux.length() + 1];
	strcpy(assignmentVariable, string_aux.c_str());

	numJoints = _numJoints.get();


    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;


	pyModule = motionPlanner->initPython(scriptFile);

	motionPlanner->sizePyArray(sizePath, pathVariable, pyModule);

	// Rover path
	double *aRoverPath = new double[sizePath*3];	
	motionPlanner->returnPyArrayDouble(2,pathVariable,aRoverPath, pyModule);

	// Rover heading
	double *aRoverHeading = new double[sizePath*1];	
	motionPlanner->returnPyArrayDouble(1,headingVariable,aRoverHeading, pyModule);

	// Trajectory
	std::vector<base::Waypoint> roverPath(sizePath);

	for(int i=0;i<sizePath;i++)
	{
		roverPath[i].position = {aRoverPath[i*3],aRoverPath[i*3+1],aRoverPath[i*3+2]};
		roverPath[i].heading = aRoverHeading[i];
	}
	

	// Joints
	double *aJoints = new double[sizePath*numJoints];	
	motionPlanner->returnPyArrayDouble(2,jointsVariable,aJoints, pyModule);

	std::vector<double> joints(sizePath*numJoints);
	memcpy(&joints[0],&aJoints[0],sizePath*numJoints*sizeof(double));


	// Assignment
	int *aAssignment = new int[sizePath*1];
	motionPlanner->returnPyArrayInt(1,assignmentVariable, aAssignment, pyModule);
	
	std::vector<int> assignment(sizePath);
	memcpy(&assignment[0],&aAssignment[0],sizePath*sizeof(int));

			

	motionPlanner->shutDownPython(pyModule);

	std::cout<<"Sending outputs...";

	_roverPath.write(roverPath);
	_joints.write(joints);
	_assignment.write(assignment);
	_sizePath.write(sizePath);

	std::cout<<" done"<<std::endl;
	
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}



	


