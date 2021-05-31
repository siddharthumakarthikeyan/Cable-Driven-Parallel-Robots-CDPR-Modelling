#ifndef CDPR_H
#define CDPR_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <visp/vpHomogeneousMatrix.h>

class CDPR
{
public:
    CDPR(ros::NodeHandle &_nh);

    inline bool ok() {return cables_ok && platform_ok && trajectory_ok ;}

    inline void setDesiredPose(double x, double y, double z, double tx, double ty, double tz)
        {Md_ = vpHomogeneousMatrix(x,y,z,tx,ty,tz);}
    inline void getPose(vpHomogeneousMatrix &M) {M = M_;}
    inline void getVelocity(vpColVector &v) {v = v_;}
    inline void getDesiredPose(vpHomogeneousMatrix &M) {M = Md_;}
    inline vpPoseVector getPoseError() { return vpPoseVector(M_.inverse()*Md_);}
    inline vpPoseVector getDesiredPoseError(vpHomogeneousMatrix &M_p, vpHomogeneousMatrix &M_c) {return vpPoseVector(M_c.inverse()*M_p);}

    inline void getDesiredVelocity(vpColVector &v) {v = v_d;}
    inline void getDesiredAcceleration(vpColVector &a) {a = a_d;}

    void sendTensions(vpColVector &f);

    // get model parameters
    inline unsigned int n_cables() {return n_cable;}
    inline double mass() {return mass_;}
    inline vpMatrix inertia() {return inertia_;}
    inline void tensionMinMax(double &fmin, double &fmax) {fmin = f_min; fmax = f_max;}

    // structure matrix
    //void computeW(vpMatrix &W);
    void computeW(vpMatrix &W);
    void computeDesiredW(vpMatrix &Wd);
    void computeLength(vpColVector &L);
    void computeDesiredLength(vpColVector &Ld);
protected:
    // subscriber to gazebo data
    ros::Subscriber cables_sub, platform_sub;
    bool cables_ok, platform_ok, trajectory_ok;
    sensor_msgs::JointState cable_states;

    // subscriber to desired pose
    ros::Subscriber setpoint_sub, desiredVel_sub, desiredAcc_sub;

    // publisher to tensions
    ros::Publisher tensions_pub;
    sensor_msgs::JointState tensions_msg;
    

    // pf pose and velocity
    vpHomogeneousMatrix M_, Md_;
    vpColVector v_, v_d, a_d;

    // model data
    double mass_, f_min, f_max;
    vpMatrix inertia_;
    std::vector<vpTranslationVector> Pf, Pp;
    unsigned int n_cable;


    // callback for platform state
    void PFState_cb(const gazebo_msgs::LinkStateConstPtr &_msg)
    {
        platform_ok = true;
        M_.insert(vpTranslationVector(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z));
        M_.insert(vpQuaternionVector(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z,_msg->pose.orientation.w));
        v_.resize(6);
        v_[0]=_msg->twist.linear.x; v_[1]=_msg->twist.linear.y; v_[2]=_msg->twist.linear.z; 
        v_[3]=_msg->twist.angular.x;  v_[4]=_msg->twist.angular.y; v_[5]=_msg->twist.angular.z;
    }

    // callback for pose setpoint
    void Setpoint_cb(const geometry_msgs::PoseConstPtr &_msg)
    {
        Md_.insert(vpTranslationVector(_msg->position.x, _msg->position.y, _msg->position.z)); 
        Md_.insert(vpQuaternionVector(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z,_msg->orientation.w));
    }

    // callback for cable states
    void Cables_cb(const sensor_msgs::JointState &_msg)
    {
        cables_ok = true;
        cable_states = _msg;
    }

    void DesiredVel_cb(const geometry_msgs::TwistConstPtr &_msg)
    {   
        trajectory_ok=true;
        v_d.resize(6);
        v_d[0]=_msg->linear.x; v_d[1]=_msg->linear.y; v_d[2]=_msg->linear.z;
        v_d[3]=_msg->angular.x; v_d[4]=_msg->angular.y; v_d[5]=_msg->angular.z;       
    }

    void DesiredAcc_cb(const geometry_msgs::TwistConstPtr &_msg)
    {
        a_d.resize(6);
        a_d[0]=_msg->linear.x; a_d[1]=_msg->linear.y; a_d[2]=_msg->linear.z;
        a_d[3]=_msg->angular.x; a_d[4]=_msg->angular.y; a_d[5]=_msg->angular.z;
    }
};

#endif // CDPR_H
