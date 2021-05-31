#ifndef trajectory_H
#define trajectory_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <visp/vpHomogeneousMatrix.h>
#include <string>

using std::endl;
using std::cout;
using std::string;

class Trajectory
{
        public:
        Trajectory(ros::NodeHandle &_node)
        {    
                // publisher to setpoints
                setpointPose_pub = _node.advertise<geometry_msgs::Pose>("pf_setpoint", 1);
                setpointVel_pub = _node.advertise<geometry_msgs::Twist>("desired_vel",1);
                setpointAcc_pub = _node.advertise<geometry_msgs::Twist>("desired_acc",1);

                // load model parameters
                ros::NodeHandle Tra(_node, "Tra");

                // getparam the  working period
                Tra.getParam("time/t0", t0);
                Tra.getParam("time/t1", t1);

                //  initialize the parameter for s-curves 
                XmlRpc::XmlRpcValue element_A,element_B;
                Tra.getParam("position/A", element_A);
                Tra.getParam("position/B", element_B);


                xi.resize(3);
                xf.resize(3);
                for (unsigned int i=0;i<3;++i)
                {
                    xi[i]=element_A[i];
                    xf[i]=element_B[i];
                }
        }

        inline void InitializeTime(double &t_0, double &t_1) {t_0= t0; t_1= t1;}
        inline void InitializePose(vpRowVector &x_i, vpRowVector &x_f) {x_i = xi; x_f = xf;}

        inline vpMatrix getLmatrix(double &t)
        {
            vpMatrix M;
            M.resize(6,6);
            M[0][5]=1; M[1][4]=1; M[2][3]=2;
            M[3][0]=t*t*t*t*t; M[3][1]=t*t*t*t; M[3][2]=t*t*t; M[3][3]=t*t; M[3][4]=t; M[3][5]=1;
            M[4][0]=5*t*t*t*t; M[4][1]=4*t*t*t; M[4][2]=3*t*t; M[4][3]=2*t; M[4][4]=1; 
            M[5][0]=20*t*t*t; M[5][1]=12*t*t; M[5][2]=6*t; M[5][3]=2;  
            return M;
        }
        // get the desired position xyz
        inline vpRowVector getposition(double t, vpMatrix a)
        { 
            vpRowVector position, l;
            l.resize(6);
            l[0]=t*t*t*t*t; l[1]=t*t*t*t; l[2]=t*t*t; l[3]=t*t; l[4]=t; l[5]=1;
            position=l*a;
            return position;
        }
        //get the desired velocity
        inline vpRowVector getvelocity(double t, vpMatrix a)
        { 
            vpRowVector l;
            l.resize(6);
            l[0]=5*t*t*t*t; l[1]=4*t*t*t; l[2]=3*t*t; l[3]=2*t; l[4]=1; 
            vpRowVector vel=l*a;
            return vel;
        }

        inline vpRowVector getacceleration(double t, vpMatrix a)
        { 
            vpRowVector l;
            l.resize(6);
            l[0]=20*t*t*t; l[1]=12*t*t; l[2]=6*t; l[3]=2;  
            vpRowVector acc=l*a;
            return acc;
        }

        void sendDesiredpara(vpColVector p, vpColVector v, vpColVector acc)
        {
                // write desired pose 
                pf_d.position.x=p[0],pf_d.position.y=p[1],pf_d.position.z=p[2];
                pf_d.orientation.x=0; pf_d.orientation.y=0; pf_d.orientation.z=0; pf_d.orientation.w=1;
                // write published velocity 
                vel_d.linear.x=v[0],vel_d.linear.y=v[1],vel_d.linear.z=v[2];
                vel_d.angular.x=0,vel_d.angular.y=0,vel_d.angular.z=0;
                // write desired acceleration
                acc_d.linear.x=acc[0],acc_d.linear.y=acc[1],acc_d.linear.z=acc[2];
                acc_d.angular.x=0; acc_d.angular.y=0; acc_d.angular.z=0;
                // publish the messages 
                setpointPose_pub.publish(pf_d);
                setpointVel_pub.publish(vel_d); 
                setpointAcc_pub.publish(acc_d);
        }

        protected:
        // publisher to the desired pose , volecity and acceleration
        ros::Publisher setpointPose_pub, setpointVel_pub, setpointAcc_pub;
        geometry_msgs::Twist vel_d, acc_d; 
        geometry_msgs::Pose pf_d;

        // model parameter
        double t0, t1;
        vpRowVector  xi, xf;
};

#endif // trajectory_H
