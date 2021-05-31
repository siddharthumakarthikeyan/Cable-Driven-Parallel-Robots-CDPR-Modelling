#ifndef CABLEDRIVENPARALLELROBOT_H
#define CABLEDRIVENPARALLELROBOT_H


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
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetLinkState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>


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
#include <qpOASES.hpp>
#include "cdpr3/SOLVEQP.h"

using namespace Eigen;
using namespace std;
namespace gazebo

{
USING_NAMESPACE_QPOASES


class CableDrivenParallelRobotPlugin : public ModelPlugin
{
public:
    CableDrivenParallelRobotPlugin() {}
    ~CableDrivenParallelRobotPlugin()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_event_);
        rosnode_.shutdown();
        //    delete rosnode_;
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();
    void TrajectoryCommandCallBack(std_msgs::Float32MultiArrayConstPtr  _msg);
 void TrajectoryCommandCallBackRS(const trajectory_msgs::JointTrajectoryPointConstPtr  &_msg);
void TrajectoryCommandCallBackMA(const std_msgs::Float32MultiArrayConstPtr _msg);

    vector<vpMatrix>  Q_Decompose_Lapack(vpMatrix Matri) const;
    void computeVinit();
    vpColVector staticEquilibrum();
    vpColVector ActivesetMethod();

// struct used for solving advanced closed form by pott
 struct closeData{
 vpColVector fa_dash;
vpColVector fmin_dash;
vpMatrix W_Matrix_dash;
vpColVector w_dash;
vpColVector fmean_dash;
int ind;
 };





private:
// parse a Vector3 string
    void ReadVector3(const std::string &_string, math::Vector3 &_vector);
 
   
    // parse received joint command (joint states)
    void JointCommandCallBack(const sensor_msgs::JointStateConstPtr _msg)
    {
        if(!control_joints_)
            return;
        // store received joint state
        joint_command_ = *_msg;
        joint_command_received_ = true;
    }
   
    
    // Function to compute the desired force to be applied as soon as we recieve the trajectory
   // void TrajectoryCommandCallBack(const trajectory_msgs::JointTrajectoryPoint &_msg)
    // parse switch service
  
    
        // Function to compute the desired force to be applied as soon as we recieve the trajectory 
    vector<vpTranslationVector> GetLink(vector<string> link_name);

    
 // -- general data ----------------------------------------
    std::string robot_namespace_;
    ros::NodeHandle rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_event_;
    ros::ServiceServer switch_service_;
    bool controller_is_running_;
    double update_T_;

    // -- body control ----------------------------------------
    // model body data
    physics::LinkPtr body_;
    bool control_body_;


    // subscriber
    ros::Subscriber body_command_subscriber_, control_error_subscriber_, control_MATLAB_subscriber_, control_ROS_subscriber_ ; 
    std::string body_command_topic_;
    Eigen::VectorXd body_command_;
    bool body_command_received_;

    // -- joint control ----------------------------------------
    // model joint data
    std::vector<physics::JointPtr> joints_;
    bool control_joints_;

    // subscriber
    ros::Subscriber joint_command_subscriber_;
    std::string joint_command_topic_;
    sensor_msgs::JointState joint_command_;
    bool joint_command_received_;

    // -- publishers ----------------------------------------

    // publisher to joint state
    ros::Publisher joint_state_publisher_ ,  platform_state_publisher_,   platform_force_publisher_;
    sensor_msgs::JointState joint_states_;
    double t_prev_;

    
   // members for the controller
  
    vpRowVector n_k, n_l; 
    vpTranslationVector  desired_position,  desired_velocity,  desired_acceleration,  current_position,  current_velocity, global_frame_trans;
    vpRotationMatrix  current_rotation , desired_rotation , desired_rotation_d,desired_rotation_dd;
    vector<vpColVector>  vertices;
    vpThetaUVector current_velocity_d;
    vector<vpTranslationVector> frame_point,anchor_point; 
    vpMatrix T, Kp, Kd,T_Kernel, n_Matrix, pros, pross, index_matrix, T_reduced; 
    vpColVector n_i_transpose,  platform_position, g, platform_velocity, xdd,f_reduced, f_v3, w_reduced, x_error, b_vector,forcev,u_f, feasible_Index_set,v_ij,v_li, v_f, x_dd_error,u, w_vector, fmax, fmin, f_v,f, f_m, f_v2, initial_vertice, b_ij, v_init,next_vertex, w;
    // service objects for getting the link pose
    
    double mass,x,y,z, alpha_l, b_init, b_l;
    gazebo::math::Pose platform_pose, peace;
nav_msgs::Odometry platform_state;    
//geometry_msgs::Pose platform_state;
    geometry_msgs::Twist platform_twist;
    //geometry_msgs::Point platform_state;
    gazebo::math::Vector3 platform_vel, platform_ang;
    ros::Publisher platform_twist_publisher_;
    //geometry_msgs::Quaternion applied_force;
std_msgs::Float64MultiArray applied_force;
    std::string trajectory_command_topic, forceMatlab_command_topic;
    trajectory_msgs::JointTrajectoryPoint command_msg;
    char param[FILENAME_MAX];
    bool culprit, sommet_trouve, index_included,foundNegative, acfcheck,static_control;
    int i_i, j_j, l_l, k_k, i_init, j_init,n_s_n, found_summit, min_index, final_index ;
    vector<int> pose_counter, indexes, other_indexes;
    vector<double> I_ij, alpha, lambda;
    ros::ServiceClient solverClient; 
    int countCables;


};
GZ_REGISTER_MODEL_PLUGIN(CableDrivenParallelRobotPlugin)
}
#endif // CABLEDRIVENPARALLELROBOT_H
