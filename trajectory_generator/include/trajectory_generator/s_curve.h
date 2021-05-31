#ifndef trajectory_S_H
#define trajectory_S_H

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
                Tra.getParam("time/t4", t4);
                 t2=t4/2;
                 t3=t4-t1;

                 // getparam
                 Tra.getParam("parameter/ab", ab);
                 Tra.getParam("parameter/vb", vb);
                 Tra.getParam("parameter/h_b", h_b);
                 Tra.getParam("parameter/h_c", h_c);
                 Tra.getParam("parameter/w", w);
                 Tra.getParam("parameter/l", l);

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

        inline void InitializeTime(double &t_0, double &t_1, double &t_2, double &t_3, double &t_4) {t_0 = t0; t_1 = t1; t_2 = t2; t_3= t3; t_4 = t4;}
        inline void InitializePose(vpColVector &x_i, vpColVector &x_f) {x_i = xi; x_f = xf;}
        inline void InitializeParam(double &W, double &L, double &hc, double &hb){W=w; L=l; hc=h_c; hb=h_b;}



        inline vpMatrix getLmatrix(double &t_, double &t)
        {
                vpMatrix M;
                M.resize(6,6);
                M[0][0]=t_*t_*t_*t_*t_; M[0][1]=t_*t_*t_*t_; M[0][2]=t_*t_*t_; M[0][3]=t_*t_; M[0][4]=t_; M[0][5]=1;
                M[1][0]=5*t_*t_*t_*t_; M[1][1]=4*t_*t_*t_; M[1][2]=3*t_*t_; M[1][3]=2*t_; M[1][4]=1; 
                M[2][0]=20*t_*t_*t_; M[2][1]=12*t_*t_; M[2][2]=6*t_; M[2][3]=2; 
              
                M[3][0]=t*t*t*t*t; M[3][1]=t*t*t*t; M[3][2]=t*t*t; M[3][3]=t*t; M[3][4]=t; M[3][5]=1;
                M[4][0]=5*t*t*t*t; M[4][1]=4*t*t*t; M[4][2]=3*t*t; M[4][3]=2*t; M[4][4]=1; 
                M[5][0]=20*t*t*t; M[5][1]=12*t*t; M[5][2]=6*t; M[5][3]=2;  
                return M;
        }

        inline  void getS( vpColVector &S1, vpColVector &S2, vpColVector &S3, vpColVector &S4, vpColVector &S5, vpColVector &S6) 
        {
            S1.resize(6);S2.resize(6);S3.resize(6);S4.resize(6);S5.resize(6);S6.resize(6);

            S1[3]=1; S1[4]=vb/h_b; S1[5]= ab/h_b;
       
            S2[1]=vb/(h_c-h_b); S2[2]=ab/(h_c-h_b); S2[3]= 1;

            S3[3]=1; S3[4]=vb/(h_c-h_b); S3[5]= -ab/(h_c-h_b);
      
            S4[1]=vb/h_b; S4[2]= -ab/h_b; S4[3]=1;
     
            S5[3]=1;
       
            S6[3]=1;
        }

        // get the desired position xyz
        inline double getposition(double t, vpColVector a)
        { 
            vpRowVector l;
            l.resize(6);
            l[0]=t*t*t*t*t; l[1]=t*t*t*t; l[2]=t*t*t; l[3]=t*t; l[4]=t; l[5]=1;
            double position= l*a;
            return position;
        }
        //get the desired velocity
        inline double getvelocity(double t, vpColVector a)
        { 
            vpRowVector l;
            l.resize(6);
             l[0]=5*t*t*t*t; l[1]=4*t*t*t; l[2]=3*t*t; l[3]=2*t; l[4]=1; 
            double vel=l*a;
            return vel;
        }

        inline double getacceleration(double t, vpColVector a)
        { 
            vpRowVector l;
            l.resize(6);
            l[0]=20*t*t*t; l[1]=12*t*t; l[2]=6*t; l[3]=2;  
            double acc=l*a;
            return acc;
        }

        void sendDesiredpara(vpColVector p, vpColVector v, vpColVector acc)
        {
            // write effort to jointstate
              
            pf_d.position.x=p[0],pf_d.position.y=p[1],pf_d.position.z=p[2];
            pf_d.orientation.w=1;
            vel_d.linear.x=v[0],vel_d.linear.y=v[1],vel_d.linear.z=v[2];
            acc_d.linear.x=acc[0],acc_d.linear.y=acc[1],acc_d.linear.z=acc[2];

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
        double t0, t1, t2, t3, t4, vb, ab, h_c, h_b, w, l;
        vpColVector  xi, xf;
};
#endif // trajectory_S_H
