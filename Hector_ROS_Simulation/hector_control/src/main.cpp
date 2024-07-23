#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <thread>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"


#include "../third_party/legged_controllers/include/legged_controllers/TargetTrajectoriesPublisher.h"
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

void runFSMController(FSM* _FSMController)
{
    ros::Rate rate(1000); 
    while (running)
    {
        _FSMController->run();
        rate.sleep();
    }
}




namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
}  // construct an unnamed namespace. Variables and functions declared within this namespace are only accessible within the same source file. 

/**
 * Helper function: computes how long it takes to achieve the desired amount of delta displacement and delta rotation (whichever the longer)
 * Note: displacement is in x, y direction only, rotation is in yaw (x axis) only
 */
scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

/**
 * A helper function which computes `TargetTrajectories`
 * @param [in] targetPose: vector_t(6), target pose (position and rotation)
 * @param [in] observation: SystemObservation 
 * @param [in] targetReachingTime: scalar_t, wall-clock time when target pose is reached
 * 
 * @param [out] TargetTrajectories: an object that encapsulates time trajectory (start and end time point),  
 *                                  state trajectory (state vector for start and end time point), 
 *                                  input trajectory (input vector for start and end time point).
 *  
 */ 
TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  
  // Initialize wall-clock time ponits of the desired trajectory: start and end time point
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // Extract current pose (position and orientation) from `observation`
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = COM_HEIGHT;
  currentPose(4) = 0;
  currentPose(5) = 0;

  // Initialize state trajectories to zero: a vector of start states and end states (i.e. a vector of 2 elements, each element is a vector of zeros)       
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));  // Recall timeTrajectory is a vector of 2 elements (start, end time ponit)
  
  // Modify start and end state trajectories by joining vectors together: size = 6 + 6 + 12
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};  // return {, , ,}: construct and return TargetTrajectories object initialized with these 3 variables.
}


/**
 * Define a callable (to be referenced in an `TargetTrajectoriesPublisher` object ): 
 * compute `TargetTrajectories` given goal pose and observation 
 */
TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  // Extract current pose (position x, y, z and orientation yaw, pitch, roll) from `observation` object:  
  const vector_t currentPose = observation.state.segment<6>(6);
  
  // Compute target end pose based on goal pose:
  const vector_t targetPose = [&]() {  // defines a lambda function: [] is lambda-introducer, [&] means all variables in the enclosing scope are accesible by reference  
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;  // set target end height = COM_HEIGHT
    target(3) = goal(3);  // 
    target(4) = 0;  // set target end rotation around y and z axis to 0
    target(5) = 0;
    return target;
  }();  // immediately invoke the lambda function () to initialize `targetPose`

  // Compute at what wall-time does the robot reach the end target pose:
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

  // Compute target trajectories 
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}


/**
 * Define another callable (to be referenced in an `TargetTrajectoriesPublisher` object ): 
 * compute `TargetTrajectories` given command velocity pose and observation 
 */

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  // Extract current pose (position and orientation) from `observation` object
  const vector_t currentPose = observation.state.segment<6>(6);  // block operations for Eigen::Matrix (a 1 col vector): extracts a segment containing 6 elements, starting at position indexed 6.
     
  // Extract yaw, pitch, roll from current pose, and convert them into rotation matrix. 
  // Then compute transformed linear velocities after applying the rotation defined by the rotation matrix.       
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);  // yaw, pitch, roll represent rotation angle around z, y, x axis respectively 
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  // Compute target end pose based on `cmdVel`  
  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {  // defines a lambda function 
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;  // 
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;  // new orientation
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // Compute at what wall-time does the robot reach the end target pose:
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  
  // Compute target trajectories 
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  
  // Modify stateTrajectories of the resultant target trajectories:
  // Set their first 3 elements of each time point = cmdVelRot
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}



int main(int argc, char ** argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    
    std::string robot_name = "hector";
    std::cout << "robot name " << robot_name << std::endl;

    ioInter = new CheatIO(robot_name);
    ros::Rate rate(1000);

    double dt = 0.001;
    Biped biped;
    // biped.setBiped();

    LegController* legController = new LegController(biped);
    // contains public member variables:
    //     commands[2] of type LegControllerCommand, 
    //     data[2] of type LegControllerData
    // methods: 
    //     updateData(LowlevelState* state)
    //     updateCommand(LowlevelCmd* cmd) 

    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();

    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   

    /* TODO: build a new object, to publish optimized trajectory */

    std::cout << "setup state etimator" << std::endl;                                                             

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM* _FSMController = new FSM(_controlData);
    
    // TODO: add a planner, 
    // replace goalToTargetTrajectory, cmdVelToTargetTrajectory 

    // FSM in a separate thread
    std::thread fsm_thread(runFSMController, _FSMController);    
    



    // TODO: a parallel thread, for TO state only,  std::thread trajectory_optimization_thread();  Refer to legged control repo

    // Create a separate node for trajectories optimization (TO) in separate thread
    ros::NodeHandle nodeHandleTrajectoriesOptimization("trajectories_optimization");

    // Load data (TODO: passed via launch file, reference.info, and task.info)
    scalar_t TARGET_DISPLACEMENT_VELOCITY = 0.5;
    scalar_t TARGET_ROTATION_VELOCITY = 1.57;
    scalar_t COM_HEIGHT = 0.3;
    vector_t DEFAULT_JOINT_STATE(12);
    DEFAULT_JOINT_STATE << -0.2, 0.72, -1.44,  -0.2, 0.72, -1.44,  -0.2, 0.72, -1.44,  -0.2, 0.72, -1.44 ;
    scalar_t TIME_TO_TARGET = 1.0;


    TargetTrajectoriesPublisher target_pose_command(nodeHandleTrajectoriesOptimization, robot_name, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);
    // Recall signature of TargetTrajectoriesPublisher class: the last 2 arguments are callables (function) that convert cmd and observation into target trajectories

    ros::spin();   
    // Since TargetTrajectoriesPublisher has set up observation subscriber and trajectory publisher, ros::spin() enters the node into a loop 
    // where it continuously checks for and responds to incoming messages (current pose) and service requests (publish target trajectory).
    // ros:spin() runs indefinitely until the node is shut down (via signal from ROS master or user command).
     





    // TODO: do we stil need this piece?    
    // END

    signal(SIGINT, ShutDown);
    
    while(running)
    {   
        ioInter->sendRecv(cmd, state);
        rate.sleep();
    }
    
    delete _controlData;
    return 0;

}
