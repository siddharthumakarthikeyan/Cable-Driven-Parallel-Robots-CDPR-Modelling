#ifndef workspace_H
#define workspace_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <visp/vpHomogeneousMatrix.h>

using std::endl;
using std::cout;

class Workspace
{
public:
    Workspace(ros::NodeHandle &w_node)
    {   
       /* // publisher to setpoints
        setpointPose_pub = w_node.advertise<geometry_msgs::Pose>("pf_setpoint", 1);
        setpointVel_pub = w_node.advertise<geometry_msgs::Twist>("desired_vel",1);
        setpointAcc_pub = w_node.advertise<geometry_msgs::Twist>("desired_acc",1);*/

        // load model parameters
        ros::NodeHandle model(w_node, "model");

        model.getParam("platform/mass",mass_);

        // initialize the size of the basic frame
        XmlRpc::XmlRpcValue element;
        model.getParam("points", element);
        n_cable = element.size();
        double x, y, z;
        for(unsigned int i=0;i<n_cable;++i)
        {
            x = element[i]["frame"][0];
            y = element[i]["frame"][1];
            z = element[i]["frame"][2];
            Pf.push_back(vpTranslationVector(x, y, z));
            /*x = element[i]["platform"][0];
            y= element[i]["platform"][1];
            z = element[i]["platform"][2];
            Pp.push_back(vpTranslationVector(x, y, z));*/
        }
            bi=Pf[1];

        model.getParam("platform/size", element);
        size_pf.resize(3);

        for(unsigned int i=0;i<3;++i)
        {
           size_pf[i] = element[i];
          
        }   
           
       para_ok=true;
    }

    //inline void InitializeTime(double &t_i, double &t_f) {t_i = t0; t_f = t4;}

    // initialize the boundary 
    inline void getBoundary(vpColVector &b_i){b_i= bi;}
    inline vpColVector getSize(vpColVector &s){s=size_pf;}
    inline double mass() {return mass_;}

    inline bool Para_ok() {return para_ok;}


    //inline void InitializePose(vpRowVector &x_i, vpRowVector &x_f) {x_i = xi; x_f = xf;}
    //inline void InitializeGain(double &t_i, double &t_f) {t_i = Kp; t_f = Kd;}


    // publisher to the desired pose , volecity and acceleration
    protected:
    ros::Publisher setpointPose_pub, setpointVel_pub, setpointAcc_pub;
    geometry_msgs::Twist vel_d, acc_d; 
    geometry_msgs::Pose pf_d;

    // model parameter
    double mass_;
    vpColVector  size_pf, bi;
    std::vector<vpTranslationVector> Pf, Pp;
    bool para_ok = false;

};

#endif // workspace_H
