#include <geometry_msgs/Pose.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/GetLinkState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>
#include <algorithm> // for std::min and std::max


#include <visp3/core/vpMatrix.h>

#include <visp3/core/vpColVector.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

// Debug trace
#include <visp3/core/vpDebug.h>


#include <cdpr3/trajectory.h>
#include <gazebo_ros/gazebo_ros_api_plugin.h>



#include <geometry_msgs/Twist.h>



using namespace std;
using namespace gazebo;

class qrDecompose
{
private:
	vpMatrix Matri;       
public:
       qrDecompose(vpMatrix Matr): Matri(Matr){};
       vpMatrix Q_Decompose_Lapack() const;
       vpMatrix   R_Decompose_Lapack() const; 
	~qrDecompose(){};
};

