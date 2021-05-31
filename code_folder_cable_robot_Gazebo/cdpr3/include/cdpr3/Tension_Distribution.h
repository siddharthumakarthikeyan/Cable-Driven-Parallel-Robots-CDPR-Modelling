//gazebo
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/common/Plugin.hh>
#include <ok_tools/okSolveQP.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
//messages and services
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetLinkState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

//Visp 
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpSubMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpThetaUVector.h>

// Debug trace
#include <visp3/core/vpDebug.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <Eigen/Householder>
#include <Eigen/Dense>

#include <vector>
#include <iostream>
#include <cdpr3/cdpr_plugin.h>




using namespace std;

namespace gazebo
{
class TensionDistributionAlgorithmn : public CableDrivenParallelRobotPlugin
{
private:
	int Nterms;
	double* pcoefficients;
	bool foundNegative, acfcheck;
	vpColVector force;
       
public:
	
	vpColVector AdvancedClosedForm();
	bool CheckTensions(vpColVector force);
   
        
        TensionDistributionAlgorithmn(); 
	~TensionDistributionAlgorithmn();
};

}
