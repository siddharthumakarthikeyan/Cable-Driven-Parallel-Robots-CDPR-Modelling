#ifndef CDPR_PLUGIN_H
#define CDPR_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <cdpr/Tensions.h>

namespace gazebo
{

class CDPRPlugin : public ModelPlugin
{
    struct Tension
    {
        ignition::math::Vector3d force;
        ignition::math::Vector3d point;
        std::string name;
    };

public:
    CDPRPlugin() {}
    ~CDPRPlugin()
    {
        rosnode_.shutdown();
        //    delete rosnode_;
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:

    // parse received joint command (joint states)
    void JointCommandCallBack(const sensor_msgs::JointStateConstPtr &_msg)
    {
        // store received joint state
        joint_command_ = *_msg;
        command_received_ = true;
    }

    // full cable tensions
    void TensionCallBack(const cdpr::TensionsConstPtr &msg)
    {
        if(msg->names.size() != msg->direction.size() ||
           msg->names.size() != msg->tensions.size())
        {
            ROS_WARN("Received inconsistent tension dimensions");
            return;
        }
        command_received_ = true;
        for(int i = 0; i < msg->names.size(); ++i)
        {
            // look for corresponding cable in command
            auto cable = std::find_if(tension_command_.begin(),
                                      tension_command_.end(),
                                      [&](Tension &t){return t.name == msg->names[i];});
            if(cable != tension_command_.end())
            {
                cable->force.X() = msg->direction[i].x;
                cable->force.Y() = msg->direction[i].y;
                cable->force.Z() = msg->direction[i].z;
                cable->force.Normalize();
                cable->force *= std::max<double>(0, msg->tensions[i]);
            }
        }
    }

private:
    // -- general data ----------------------------------------
    ros::NodeHandle rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_event_;
    double update_T_;

    // -- joint control ----------------------------------------
    // model joint data
    std::vector<physics::JointPtr> joints_;
    double f_max;

    // subscriber
    bool sim_cables_;
    ros::Subscriber command_subscriber_;
    sensor_msgs::JointState joint_command_;
    std::vector<Tension> tension_command_;
    bool command_received_;


    // -- publishers ----------------------------------------

    // publisher to joint state
    ros::Publisher joint_state_publisher_;
    sensor_msgs::JointState joint_states_;
    double t_prev_;

    // publisher of platform position
    ros::Publisher pf_publisher_;
    gazebo_msgs::LinkState pf_state_;
    physics::LinkPtr frame_link_, platform_link_;
};
GZ_REGISTER_MODEL_PLUGIN(CDPRPlugin)
}
#endif // CDPR_PLUGIN_H
