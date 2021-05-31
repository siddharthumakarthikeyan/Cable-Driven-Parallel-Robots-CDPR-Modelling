#include <log2plot/logger.h>
#include <trajectory_generator/spin_tra.h>
#include <visp/vpIoTools.h>
#include <math.h>


//-------------------------------------------------------------------------------------------------------------
// spin trajectory is based on 5 order polynomial motion planning algorith
// This is the trajectory generator to generate continuous spin curve
//---------------------------------------------------------------------------------------------------------
using namespace std;
using namespace log2plot;

int main(int argc, char ** argv)
{
    cout.precision(3);
    // init ROS node
    ros::init(argc, argv, "spin_tra");
    ros::NodeHandle node;

    Trajectory path(node);
    std::string dir = "/home/" + vpIoTools::getUserName() + "/Results/cdpr/";
    // plotting 3D figures
    vpPoseVector pose;
    Logger logger(dir);
    // save pose as 3D plot
    // the saved variable is the world pose in camera frame, we want to plot the invert
    logger.save3Dpose(pose, "trajectory", "box pose", false);

    double alpha, u, beta, beta1, beta2;

    // initialization
    vpColVector x_i(3), x_f(3);
    vpColVector P, Vel, Acc, Ax, Ay, Az, Xx,Xy,Xz;
    double t_0, t_1,t;
    vpMatrix T;
    T.resize(6,6); 

    P.resize(6); Vel.resize(6); Acc.resize(6);

    path.InitializeTime(t_0, t_1);
    path.InitializePose(x_i,x_f);
    path.InitializeParameter( alpha, beta, u);

    //beta1= (x_i[0]-x_f[0])/((x_f[2]-x_i[2])*cos(t_1*u));
    //beta2= (x_i[1]-x_f[1])/((x_f[2]-x_i[2])*sin(t_1*u));

    //  obtain the time matrix
    T=path.getTmatrix(t_0,t_1);
    // gain condition vector
    path.getX(Xx, Xy, Xz);
    
    // coefficient  matrix 
    Ax= T.inverseByLU()*Xx;
    Ay= T.inverseByLU()*Xy;
    Az= T.inverseByLU()*Xz;


    double dt = 0.01;
    ros::Rate loop(1/dt);
    int num=0, inter=0;
    num= t_1/dt;

  while (ros::ok())
  {

        cout << "--------------------------trajectory------------------" << endl; 
         // relative time from the start
         t=t_0+inter*dt;

        // Spin trajectory generator with variational radius 
        if (inter<= num)
             {
                        // desired setpoints pose
                        P[2]=x_i[2] + alpha*path.getS(t,Az); 
                        P[0]= x_i[0] + beta*(P[2]-x_i[2])*cos(u*path.getS(t,Ax)); 
                        P[1]= x_i[1] + beta*(P[2]-x_i[2])*sin(u*path.getS(t,Ay)); 
                        // desired velocity computation
                        Vel[2]= alpha*path.getSdot(t, Az); 
                        Vel[0]= - beta* (P[2]-x_i[2]) *sin(u*path.getS(t,Ax))*path.getSdot(t, Ax) ; 
                        Vel[1]=  beta* (P[2]-x_i[2])*cos(u*path.getS(t,Ay)) *path.getSdot(t, Ay);
                        // desierd acceleration computation
                        Acc[2]= alpha*path.getSddot(t, Az);  
                        Acc[0]= -beta*(P[2]-x_i[2]) *( cos(u*path.getS(t,Ax)) *path.getSdot(t, Ax) *path.getSdot(t, Ax)+sin(u*path.getS(t,Ax))*path.getSddot(t, Ax)); 
                        Acc[1]= -beta* (P[2]-x_i[2])*(sin(u*path.getS(t,Ax)) *path.getSdot(t, Ay)*path.getSdot(t, Ay)-cos(u*path.getS(t,Ax))*path.getSddot(t, Ay));
                 }
                // Spin trajectory generator without variational radius 
/*                if (inter<= num)
                 {
                    // desired setpoints pose
                    P[2]=x_i[2] + alpha*path.getS(t,Az); 
                    P[0]= x_i[0] + beta*cos(u*path.getS(t,Ax)); 
                    P[1]= x_i[1] + beta*sin(u*path.getS(t,Ay)); 
                    // desired velocity computation
                    Vel[2]= alpha*path.getSdot(t, Az); 
                    Vel[0]= - beta*sin(u*path.getS(t,Ax))*path.getSdot(t, Ax) ; 
                    Vel[1]=  beta*cos(u*path.getS(t,Ay)) *path.getSdot(t, Ay);
                    // desierd acceleration computation
                    Acc[2]= alpha*path.getSddot(t, Az);  
                    Acc[0]= -beta*( cos(u*path.getS(t,Ax)) *path.getSdot(t, Ax) *path.getSdot(t, Ax)+sin(u*path.getS(t,Ax))*path.getSddot(t, Ax)); 
                    Acc[1]= -beta*(sin(u*path.getS(t,Ax)) *path.getSdot(t, Ay)*path.getSdot(t, Ay)-cos(u*path.getS(t,Ax))*path.getSddot(t, Ay));
                 }*/

            /*    // compute desied parameter
            if (inter<= num)
            {
                P[2]=x_i[2] + alpha*path.getS(t,Az); P[0]= x_i[0] +beta1*(P[2]-x_i[2])*cos(u*t); P[1]= x_i[1] + beta2*(P[2]-x_i[2])*sin(u*t); 

                Vel[2]= alpha*path.getSdot(t, Az); Vel[0]= beta1*Vel[2]*cos(u*t) - beta1* (P[2]-x_i[2]) *u*sin(u*t); 
                Vel[1] =  beta2* Vel[2]*sin(u*t) + u*beta2*(P[2]-x_i[2])*cos(u*t);
                // compute acceleration
                Acc[2]= alpha*path.getSddot(t, Az);  Acc[0]= beta1*Acc[2]*cos(u*t)- u*beta1*Vel[2]*sin(u*t)-u*u*beta1*(P[2]-x_i[2]) *cos(u*t);  
                Acc[1]= beta2* Acc[2]*sin(u*t)+u*beta2*Vel[2]*cos(u*t) - u*u*beta2*(P[2]-x_i[2])*sin(u*t);
            }*/

        path.sendDesiredpara(P, Vel, Acc) ;
        pose.buildFrom(P[0], P[1],  P[2],  P[3],  P[4],  P[5]);

        // log
        logger.update();

        cout << " Desired position: "<< P.t() <<endl;
        cout << " Desired velocity:" << Vel .t()<<endl;
        cout << " Desired acceleration:" << Acc.t() <<endl;
        inter++;

        ros::spinOnce();
        loop.sleep();
  }

  logger.plot("", true);
  return 0;
};
