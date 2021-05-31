
#include <ros/ros.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ros/node_handle.h>
#include <cdpr/cdpr_plugin.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

void CDPRPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get model and name
    model_ = _model;

    // register ROS node & time
    rosnode_ = ros::NodeHandle();
    t_prev_ = 0;

    // *** JOINT CONTROL
    joints_.clear();
    tension_command_.clear();
    rosnode_.param("/model/sim_cables", sim_cables_, false);

    if(model_->GetJointCount() != 0 && sim_cables_)
    {
        // initialize subscriber to joint commands
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    "cable_command", 1,
                    boost::bind(&CDPRPlugin::JointCommandCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        command_subscriber_ = rosnode_.subscribe(ops);
        command_received_ = false;

        // setup joint states
        std::vector<std::string> joint_names;
        std::string name;
        physics::JointPtr joint;

        for(unsigned int i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();

            if(name.find("cable") == 0) // we got a cable
            {
                joints_.push_back(joint);
                // save name
                joint_names.push_back(name);
                // get maximum effort
                f_max = joint->GetEffortLimit(0);
            }
        }

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("cable_states", 1);
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
        joint_states_.effort.resize(joints_.size());
    }
    else
    {
        // read attach points from param
        XmlRpc::XmlRpcValue p;
        rosnode_.getParam("/model/points", p);
        Tension t;
        for(int i = 0; i < p.size(); ++i)
        {
            for(auto elem: p[i])
            {
                std::stringstream ss;
                ss << "cable" << i;
                t.name = ss.str();
                t.point.X() = elem.second[0];
                t.point.Y() = elem.second[1];
                t.point.Z() = elem.second[2];
            }
            tension_command_.push_back(t);
        }

        // init subscriber
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<cdpr::Tensions>(
                    "cable_command", 1,
                    boost::bind(&CDPRPlugin::TensionCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        command_subscriber_ = rosnode_.subscribe(ops);
        command_received_ = false;
    }

    // *** END JOINT CONTROL

    // get frame and platform links
    for(auto &link: model_->GetLinks())
    {
        if(link->GetName() == "frame")
            frame_link_ = link;
        else if(link->GetName() == "platform")
            platform_link_ = link;
    }

    // setup platform state publisher
    pf_publisher_ = rosnode_.advertise<gazebo_msgs::LinkState>("pf_state",1);
    pf_state_.link_name = "platform";
    pf_state_.reference_frame = "frame";

    // store update rate
    if(_sdf->HasElement("updateRate"))
        update_T_ = 1./_sdf->Get<double>("updateRate");
    else
        update_T_ = 0;

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CDPRPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started CDPR Plugin for %s.", _model->GetName().c_str());
}

void CDPRPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();

    // deal with joint control
    if(command_received_)
    {
        if(sim_cables_)
        {
            physics::JointPtr joint;
            unsigned int idx;
            for(unsigned int i=0;i<joint_command_.name.size();++i)
            {
                // find corresponding model joint
                idx = 0;
                while(joint_states_.name[idx] != joint_command_.name[i])
                    idx++;
                joint = joints_[idx];
                // only apply positive tensions
                if(joint_command_.effort[i] > 0)
                    joint->SetForce(0,std::min(joint_command_.effort[i], f_max));
            }
        }
        else
        {
#if GAZEBO_MAJOR_VERSION < 9
            auto rot = platform_link_->GetWorldPose().rot;
#else
            auto rot = platform_link_->WorldPose().Rot();
#endif
            //rot.Invert(); ?? to check
            for(const auto &t: tension_command_)
                platform_link_->AddForceAtRelativePosition(rot*t.force, t.point);
        }
    }

    // publish joint states
    double t = ros::Time::now().toSec();
    if((t-t_prev_) > update_T_ && joints_.size() != 0)
    {
        t_prev_ = t;
        joint_states_.header.stamp = ros::Time::now();

        for(unsigned int i=0;i<joints_.size();++i)
        {
#if GAZEBO_MAJOR_VERSION < 9
            joint_states_.position[i] = joints_[i]->GetAngle(0).Radian();
#else
            joint_states_.position[i] = joints_[i]->Position();
#endif
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
            joint_states_.effort[i] = joints_[i]->GetForce(0);
        }
        joint_state_publisher_.publish(joint_states_);
    }

    // publish pf state
#if GAZEBO_MAJOR_VERSION < 9
    math::Pose pf_pose = platform_link_->GetWorldPose() - frame_link_->GetWorldPose();
    pf_state_.pose.position.x = pf_pose.pos.x;
    pf_state_.pose.position.y = pf_pose.pos.y;
    pf_state_.pose.position.z = pf_pose.pos.z;
    pf_state_.pose.orientation.x = pf_pose.rot.x;
    pf_state_.pose.orientation.y = pf_pose.rot.y;
    pf_state_.pose.orientation.z = pf_pose.rot.z;
    pf_state_.pose.orientation.w = pf_pose.rot.w;
    math::Vector3 vel = pf_pose.rot.RotateVector(platform_link_->GetRelativeLinearVel());
    pf_state_.twist.linear.x = vel.x;
    pf_state_.twist.linear.y = vel.y;
    pf_state_.twist.linear.z = vel.z;
    vel = pf_pose.rot.RotateVector(platform_link_->GetRelativeAngularVel());
    pf_state_.twist.angular.x = vel.x;
    pf_state_.twist.angular.y = vel.y;
    pf_state_.twist.angular.z = vel.z;
#else
    auto pf_pose = platform_link_->WorldPose() - frame_link_->WorldPose();
    pf_state_.pose.position.x = pf_pose.Pos().X();
    pf_state_.pose.position.y = pf_pose.Pos().Y();
    pf_state_.pose.position.z = pf_pose.Pos().Z();
    pf_state_.pose.orientation.x = pf_pose.Rot().X();
    pf_state_.pose.orientation.y = pf_pose.Rot().Y();
    pf_state_.pose.orientation.z = pf_pose.Rot().Z();
    pf_state_.pose.orientation.w = pf_pose.Rot().W();
    auto vel = pf_pose.Rot().RotateVector(platform_link_->RelativeLinearVel());
    pf_state_.twist.linear.x = vel.X();
    pf_state_.twist.linear.y = vel.Y();
    pf_state_.twist.linear.z = vel.Z();
    vel = pf_pose.Rot().RotateVector(platform_link_->RelativeAngularVel());
    pf_state_.twist.angular.x = vel.X();
    pf_state_.twist.angular.y = vel.Y();
    pf_state_.twist.angular.z = vel.Z();
#endif



    pf_publisher_.publish(pf_state_);
    ros::spinOnce();
}

}   // namespace gazebo
