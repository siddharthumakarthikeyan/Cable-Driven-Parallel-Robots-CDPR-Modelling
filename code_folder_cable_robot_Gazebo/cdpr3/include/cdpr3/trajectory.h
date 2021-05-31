#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/JointState.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>


#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetLinkState.h>

#include <iostream>
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

using namespace std;
using namespace gazebo;
using namespace Eigen;
class Trajectory
{
private:
	int Nterms;
	double* pcoefficients;
       
public:
	
	double horner(vector<double> v, double x);
	double getposition5(vector<double> v, double x);
	double getvelocity5(vector<double> v, double x);
	double getacceleration5(vector<double> v, double x);
	void print(void);
        std::vector<double> polynomial_derivative( const std::vector<double> & array );
        vector<double> Trajectory5(double x,double xf,double xd,double xdf,double xdd,double xddf,double start_time,double end_time);
        vector<double> linspace(double min, double max, int n);
        void init( double coefficients[], int nterms );
        vector<vpTranslationVector> GetLink(vector<string> link_name);
        void  ApplyForce(vpColVector effort, vector<string> joint_names);
       
   
        
        Trajectory(); 
	~Trajectory();
};

