#include <log2plot/logger.h>
#include <trajectory_generator/s_curve.h>
#include <visp/vpIoTools.h>


/*
*---------------------------------------------------------------------------------------------------------------
*  s-curve method based on 5 order polynomial motion planning algorithm
*  This is the trajectory generator to generate continuous multiple points' trajectory
*-----------------------------------------------------------------------------------------------------------------------------
*/


using namespace std;
using namespace log2plot;

int main(int argc, char ** argv)
{
        cout.precision(3);
        // init ROS node
        ros::init(argc, argv, "s_curve");
        ros::NodeHandle node;

        Trajectory path(node);
        std::string dir = "/home/" + vpIoTools::getUserName() + "/Results/cdpr/";
        // plotting 3D figures
        vpPoseVector pose;
        Logger logger(dir);
        // save pose as 3D plot
        // the saved variable is the world pose in camera frame, we want to plot the invert
        logger.save3Dpose(pose, "trajectory", "box pose", false);


        // initialization
        vpColVector x_i(3), x_f(3), v_i(3), v_f(3), a_i(3), a_f(3);
        vpColVector P, Vel, Acc, T;
        double t_0, t_1, t_2, t_3, t_4, w, l, t, h_b, h_c;
        vpColVector A1, A2, A3, A4, A5, A6;
        vpColVector  S1, S2, S3, S4, S5, S6;
        vpMatrix W1, W2, W3, W4, W5, W6;
        W1.resize(6,6); W2.resize(6,6); W3.resize(6,6); W4.resize(6,6); W5.resize(6,6); 
        A1.resize(6);  A2.resize(6);  A3.resize(6);  A4.resize(6);
        P.resize(6); Vel.resize(6); Acc.resize(6), T.resize(5);

       path.InitializeTime(t_0, t_1, t_2, t_3,t_4);
       //  t_0=0; t_4=10; t_1=2.1; t_2=t_4/2; t_3= t_4-t_1;
        T[0]=t_0;   T[1]=t_1;   T[2]=t_2;  T[3]=t_3;   T[4]=t_4; 
        path.InitializePose(x_i,x_f);
        path.InitializeParam(w, l, h_c, h_b);

       //  obtain the time matrix
        W1=path.getLmatrix(t_0,t_1);
        W2=path.getLmatrix(t_1,t_2);
        W3=path.getLmatrix(t_2,t_3);
        W4=path.getLmatrix(t_3,t_4);
        W5=path.getLmatrix(t_1,t_3);
     
        //  obtain the desired S matrix
        path.getS(S1, S2, S3, S4, S5, S6);
        
        // coefficient  matrix
        A1= W1.inverseByLU()*S1;
        A2= W2.inverseByLU()*S2;
        A3= W3.inverseByLU()*S3;
        A4= W4.inverseByLU()*S4;
        A5= W5.inverseByLU()*S5;
        A6= W5.inverseByLU()*S6;

        double dt = 0.01;
        ros::Rate loop(1/dt);
        int num=0, inter=0;

        cout << "--------------------------------trajectory-------------------------------" << endl;
      while (ros::ok())
      {
                // relative time from the beginning
                t=t_0+inter*dt;
                // Check the time period 
                if (inter<t_1/dt)
                {
                  P[0]= x_i[0]; P[1]= x_i[1]; P[2]=h_b*path.getposition(t,A1)+x_i[2];
                  Vel[2]=h_b*path.getvelocity(t,A1);
                  Acc[2]=h_b*path.getacceleration(t,A1);
                }
                else if (inter >= t_1/dt && inter < t_2/dt)
                {
                  P[0]= x_i[0]+w*path.getposition(t,A5); P[1]= x_i[1]+l*path.getposition(t,A5); P[2]=(h_c-h_b)*path.getposition(t,A2)+h_b+x_i[2];
                  Vel[0]= w*path.getvelocity(t,A5); Vel[1]=l*path.getvelocity(t,A5); Vel[2]=(h_c-h_b)*path.getvelocity(t,A2);
                  Acc[0]=w*path.getacceleration(t,A5);  Acc[1]=l*path.getacceleration(t,A5); Acc[2]=(h_c-h_b)*path.getacceleration(t,A2);
                }
                else if (inter>= t_2/dt && inter< t_3/dt)
                {
                  P[0]= x_i[0]+w*path.getposition(t,A5); P[1]= x_i[1]+l*path.getposition(t,A5); P[2]=-(h_c-h_b)*path.getposition(t,A3)+h_c+x_i[2];
                  Vel[0]= w*path.getvelocity(t,A5); Vel[1]=l*path.getvelocity(t,A5); Vel[2]= -(h_c-h_b)*path.getvelocity(t,A3);
                  Acc[0]=w*path.getacceleration(t,A5);  Acc[1]=l*path.getacceleration(t,A5); Acc[2]= -(h_c-h_b)*path.getacceleration(t,A3);
                }
                else if (inter>= t_3/dt && inter<= t_4/dt)
                {
                  P[0]= x_f[0]; P[1]= x_f[1]; P[2]= -h_b*path.getposition(t,A4)+h_b+x_i[2];
                  Vel[0]=0;  Vel[1]=0; Vel[2]= -h_b*path.getvelocity(t,A4);
                  Acc[0]=0; Acc[1]=0; Acc[2]= -h_b*path.getacceleration(t,A4);
                }

                path.sendDesiredpara(P, Vel, Acc) ;
                pose.buildFrom(P[0], P[1],  P[2],  P[3],  P[4],  P[5]);

                // log
                logger.update();

                cout << " Desired position" << P.t() <<endl;
                cout << " Desired velocity" << Vel .t()<<endl;
                cout << " Desired acceleration" << Acc.t() <<endl;

                inter++;
                ros::spinOnce();
                loop.sleep();
       }
      logger.plot("", true);
      return 0;
};
