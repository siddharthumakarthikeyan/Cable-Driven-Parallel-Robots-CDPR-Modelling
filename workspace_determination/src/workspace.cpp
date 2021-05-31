
#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>
#include <math.h>
#include <workspace_determination/workspace.h>

using namespace std;


/*
 * Closed form controller to show input/output of the CDPR class
 *
 * Does not consider positive-only cable tensions and assumes it is more or less a UPS parallel robot
 *
 *
 */


void Param(ros::NodeHandle &nh, const string &key, double &val)
{
    if(nh.hasParam(key))
        nh.getParam(key, val);
    else
        nh.setParam(key, val);
}

int main(int argc, char ** argv)
{

    cout.precision(3);
    // init ROS node
    ros::init(argc, argv, "workspace_determination");
    ros::NodeHandle node_w;
    
    // declaration of the node
    Workspace space(node_w);


    //initiallization of the parameter
    double mass, x_b, y_b, z_b, x_s, y_s, z_s;
    vpColVector boxsize(3), B(3);



    // 

    mass = node_w.mass();
    node_w.getSize(boxsize);
    node_w.getBoundary(B);
    
    x_b=abs(B[0]);
    y_b=abs(B[1]);
    z_b=abs(B[2]);
    x_s=1/2*boxsize[0];
    y_s=1/2*boxsize[1];
    z_s=1/2*boxsize[2];

    double inter, x, y, z=0;

    dx=0.005;
    dy=0.005;
    dz=0.01;

    inter_x= 2* x_b/dx;
    inter_y= 2* y_b/dy;
    inter_z= z_b/dz;

    double dt = 0.01;
    ros::Rate loop(1/dt);
    std::vector<vpTranslationVector> P;

  
    cout << "workspace analysis ready" << fixed << endl; 
    while(ros::ok())
    {
        cout << "------------------" << endl;
        
        if ( node_w.Para_ok())
        {  
            for (unsigned int i = 0; i <= inter_y; ++i)
            {
                y=y_b-i*dy;

                for (unsigned int k = 0 ; k <= inter_x; ++k)
                {
                    x=x_b-k*dx;
                    P.push_back(vpTranslationVector(x, y, z));
                }

            }

        }
        z+=dz;

        

        
        ros::spinOnce();
        loop.sleep();
    }





}
