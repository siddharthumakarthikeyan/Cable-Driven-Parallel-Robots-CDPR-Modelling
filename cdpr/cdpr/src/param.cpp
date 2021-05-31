#include <ros/node_handle.h>
#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{

    // init ROS node
    ros::init(argc, argv, "gna");
    ros::NodeHandle nh;

    bool sim = false;
    nh.param("/model/sim_cables", sim);
    std::cout << sim << std::endl;





}
