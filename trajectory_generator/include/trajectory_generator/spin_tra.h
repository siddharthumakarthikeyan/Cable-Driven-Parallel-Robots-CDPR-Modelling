#ifndef trajectory_S_H
#define trajectory_S_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
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

                        // getparam the coefficient number
                        Tra.getParam("parameter/alpha", alpha );
                        Tra.getParam("parameter/u", u);
                        Tra.getParam("parameter/beta", beta);

                        //  initialize the parameter for s-curves 
                        XmlRpc::XmlRpcValue element_A,element_B;
                        Tra.getParam("position/A", element_A);
                        Tra.getParam("position/B", element_B);

                        // initialize the position
                        xi.resize(3);
                        xf.resize(3);
                        for (unsigned int i=0;i<3;++i)
                        {
                            xi[i]=element_A[i];
                            xf[i]=element_B[i];
                        }
              }

        inline void InitializeTime(double &t_0, double &t_1) {t_0 = t0; t_1 = t1;}
        inline void InitializePose(vpColVector &x_i, vpColVector &x_f) {x_i = xi; x_f = xf;}
        inline void InitializeParameter(double &alpha_ , double &beta_,  double &u_ ) { alpha_= alpha; beta_=beta; u_= u;}
        
        // define time matrix
        inline vpMatrix getTmatrix(double &t_, double &t)
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

        // condition matrix where define the pose, velocity and acceleration of initial point and final point
        inline  void getX( vpColVector &Xx, vpColVector &Xy, vpColVector &Xz) 
        {       

                // the beta cannot be too large, otherwise, arc cos will be larger than 1 , thus no arc value

                // n_c is the round number of spining             
                int n_c=2;
                Xx.resize(6);Xy.resize(6); Xz.resize(6);

                Xz[0]=0; 
                Xz[3]=(xf[2]-xi[2])/alpha;

                // variational radius associated with z 
                 Xx[0]= M_PI/2;
                Xx[3]= acos((xf[0]-xi[0])/ (beta*(xf[2]-xi[2])))+2*n_c*M_PI;

                Xy[0]= 0;
                Xy[3]= asin((xf[1]-xi[1])/ (beta*(xf[2]-xi[2])))+2*n_c*M_PI;

                // fixed radius of spin trajectory
/*                Xx[0]= M_PI/2;
                Xx[3]= acos((xf[0]-xi[0])/ beta)+2*n_c*M_PI;

                Xy[0]= 0;
                Xy[3]= asin((xf[1]-xi[1])/ beta)+2*n_c*M_PI;*/
        }

        // get the desired S(t)
        inline double getS(double t, vpColVector a)
        { 
            vpRowVector l;
            l.resize(6);
            l[0]=t*t*t*t*t; l[1]=t*t*t*t; l[2]=t*t*t; l[3]=t*t; l[4]=t; l[5]=1;
            double S = l*a;
            return S ;
        }
        //get the desired S(t)_dot
        inline double getSdot(double t, vpColVector a)
        { 
            vpRowVector l;
            l.resize(6);
             l[0]=5*t*t*t*t; l[1]=4*t*t*t; l[2]=3*t*t; l[3]=2*t; l[4]=1; 
            double Sdot=l*a;
            return Sdot;
        }
        //get the desired S(t)_ddot
        inline double getSddot(double t, vpColVector a)
        { 
            vpRowVector l;
            l.resize(6);
            l[0]=20*t*t*t; l[1]=12*t*t; l[2]=6*t; l[3]=2;  
            double Sddot=l*a;
            return Sddot;
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
    double t0, t1, alpha, u, beta;
    vpColVector  xi, xf;
};

#endif // trajectory_S_H
