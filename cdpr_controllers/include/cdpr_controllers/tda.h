#ifndef TDA_H
#define TDA_H

#include <cdpr_controllers/qp.h>
#include <cdpr/cdpr.h>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <ros/publisher.h>


// this class implements all candidates for TDA's
// this way we do not have the same code in all sources


class TDA
{
public:

    // how we perform the TDA
    typedef enum
    {
        minW, minT, noMin, closed_form, Barycenter, slack_v, adaptive_gains, cvxgen_slack, cvxgen_minT
    } minType;

   
    TDA(CDPR &robot, ros::NodeHandle &_nh, minType _control, bool warm_start = false);

    // will look for a solution in [tau +- dTau_max]
    void ForceContinuity(double _dTau_max) {dTau_max = _dTau_max;}
     void Weighing(double lambda){ _lambda = lambda;}

    vpColVector ComputeDistribution(vpMatrix &W, vpColVector &w);
    vpColVector ComputeDistributionG(vpMatrix &W, vpColVector &ve, vpColVector &pe, vpColVector &w );

    // for minA
    void GetAlpha(vpColVector &a)
    {
        if(control == slack_v || control== cvxgen_slack)
        {
            a[0] = x[8]; a[1] = x[9]; a[2] = x[10]; 
            a[3] = x[11]; a[4] = x[12]; a[5] = x[13];
        }
    }
    void GetGains(vpColVector &a)
    {
        if(control == adaptive_gains)
       {
            a[0] = x[8]; 
            a[1] = x[9];
            a[2] = x[10]; 
            a[3] = x[11];
        }
    }
    void Getresidual(vpColVector &a,vpColVector &e)
    {
        if(control == adaptive_gains)
        {
            a[0]=w_d[0];a[1]=w_d[1];a[2]=w_d[2];
            e[0]=w_d[3];e[1]=w_d[4];e[2]=w_d[5];
        }   
    }


protected:
    minType control;
    int n,index, num_v,iter=0;
    double tauMin, tauMax;

    vpMatrix Q, A, C;
    vpColVector r, b, d, x, w_d,wp;
    vpSubColVector tau;

    bool update_d;
    double dTau_max, dAlpha,  _lambda;

    bool reset_active;
    std::vector<bool> active;
    std::vector<vpColVector> vertices;
    
     // declaration of closed form
     vpColVector f_m, f_v, w_;
     
    // declaration of Barycenter
    double m;
    vpColVector  lambda, F, p;
    vpMatrix kerW, H, ker, ker_inv;
    // publisher to barycenter plot
    ros::Publisher bary_pub;
};

#endif // TDA_H
