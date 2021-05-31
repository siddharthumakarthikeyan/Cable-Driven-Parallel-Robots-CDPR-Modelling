#include <cdpr_controllers/tda.h>
#include <visp/vpIoTools.h>

// this script is associated with TDAs 
// note that due to the dimension problem, if run "cvxgen_minT" method, must comment "cvxgen_slack"
// and "adaptive_gains" code block, as well as uncomment relative header files.
// these code based on CVXGEN can be separated into different files in order to avoid the impact among them

// ***************************************************
//* uncomment for the slack varibles algorithm
//****************************************************
/*#include "../cvxgen_slack/solver.h"
#include "../cvxgen_slack/solver.c"
#include "../cvxgen_slack/matrix_support.c"
#include "../cvxgen_slack/util.c"
#include "../cvxgen_slack/ldl.c"*/

//************************************************************
//* minimize the cable tensions
//************************************************************
#include "../cvxgen_minT/solver.h"
#include "../cvxgen_minT/solver.c"
#include "../cvxgen_minT/matrix_support.c"
#include "../cvxgen_minT/util.c"
#include "../cvxgen_minT/ldl.c"

//************************************************************
//* adaptive gains 
//************************************************************
/*#include "../cvxgen_gains/solver.h"
#include "../cvxgen_gains/solver.c"
#include "../cvxgen_gains/matrix_support.c"
#include "../cvxgen_gains/util.c"
#include "../cvxgen_gains/ldl.c"*/

// declare the namespaces used in CVXGEN code
Vars vars;
Params params;
Workspace work;
Settings settings;


using std::cout;
using std::endl;
using std::vector;


TDA::TDA(CDPR &robot, ros::NodeHandle &_nh, minType _control, bool warm_start)
{
    // number of cables
    n = robot.n_cables();

    // mass of platform
    m=robot.mass();

    // forces min / max
    robot.tensionMinMax(tauMin, tauMax);

    // indicator of number of interaton which has infeasible  tension
    index=0;

    control = _control;
    update_d = false;

    x.resize(n);

    reset_active = !warm_start;
    cout << "reset_active" << reset_active << endl;
    active.clear();

    // prepare variables
    if(control == minT)
    {
        // min |tau|
        //  st W.tau = w        // assume the given wrench is feasible dependent on the gains
        //  st t- < tau < tau+

        // min tau
        Q.eye(n);
        r.resize(n);
        // equality constraint
        A.resize(6,n);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }

    else if(control == minW)
    {
        // min |W.tau - w|      // does not assume the given wrench is feasible
        //   st t- < tau < t+

        Q.resize(6,n);
        r.resize(6);
        // no equality constraints
        A.resize(0,n);
        b.resize(0);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if (control == slack_v)
    {
        // slack variable solved by qp solver provided by Olivier
        // raise the dimension of decision vector
        x.resize(n+6); 

        // normlise the Q matrix
        Q.eye(n+6); Q *= 1./(tauMax*tauMax);
        r.resize(n+6);
        Q[n][n]=Q[n+1][n+1]=Q[n+2][n+2]=Q[n+3][n+3]=Q[n+4][n+4]=Q[n+5][n+5]=1;

        // equality constraints
        A.resize(6,n+6);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*(n+6), (n+6));
        d.resize(2*(n+6));
        for(unsigned int i=0;i<n;++i)
        {
            // f < fmax
            C[i][i] = 1;
            // -f < -fmin
            C[i+n][i] = -1;
            d[i] = 9950;
            d[i+n] = 0;
        }
    }
    else if ( control == closed_form)
    {
        // tau=f_m+f_v
        //  f_m=(tauMax+tauMin)/2
        //  f=f_m- (W^+)(w+W*f_m)

        f_m.resize(n);
        f_v.resize(n);
        // no equality constraints
        d.resize(2*n);
        for (unsigned int i = 0; i <n; ++i)
        {
            f_m[i]=(tauMax+tauMin)/2;
            d[i] =tauMax;
            d[i+n] = - tauMin;
        }
    }
    else if ( control == Barycenter)
    {   
        // publisher to plot
        bary_pub = _nh.advertise<std_msgs::Float32MultiArray>("barycenter", 1);
        // initialize the matrix
        std::vector<double>::size_type num_v;
        H.resize(n , n-6);
        d.resize(2*n);
        lambda.resize(2);
        F.resize(2);
        // particular solution from the pseudo Inverse
        p.resize(n);
        ker.resize(n-6,n-6);
        ker_inv.resize(n-6,n-6);
        for (unsigned int i = 0; i <n; ++i)
        {
            d[i] =tauMax;
            d[i+n] = - tauMin;
        }
    }
    else if ( control == adaptive_gains)
    {
        // min lambda_1|f| +|lambda_2.(kp-kp*)|+|lambda_2.(kd-kd*)|
        //  st W.f =Ip(xdd+Kd.v_e+Kp.p_e)-wg
        //  st -10 < kp-kp* < 10
        //  st -10 < kd-kd* <10
        //  st f - < f < f +
        w_d.resize(6);

        // publisher to plot
        bary_pub = _nh.advertise<std_msgs::Float32MultiArray>("barycenter", 1);
        H.resize(n , n-6);
        // particular solution from the pseudo Inverse
        p.resize(n);
        ker.resize(n-6,n-6);
        ker_inv.resize(n-6,n-6);

        x.resize(n+4);  // x = (tau, kp, kd)
        r.resize(n+2);
        Q.eye(n+2); 

        // equality constraints
        A.resize(6,n+2);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*(n+2), (n+2));
        d.resize(2*(n+2));
        for(unsigned int i=0;i<n;++i)
        {
            d[i] = tauMax;
            d[i+n] = -tauMin;
        }
    }
    else if(control == cvxgen_slack)
    {   
        x.resize(n+6);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            d[i] = tauMax;
            d[i+n] = -tauMin;
        }
    }
    else if (control == cvxgen_minT)
    {
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            d[i] = tauMax;
            d[i+n] = -tauMin;
        }
    }
    tau.init(x, 0, n);
}



vpColVector TDA::ComputeDistribution(vpMatrix &W, vpColVector &w)
{
    if(reset_active)
        for(int i=0;i<active.size();++i)
            active[i] = false;

    if(update_d && control != noMin && control != closed_form)
    {
        for(unsigned int i=0;i<n;++i)
        {
            d[i] = std::min(tauMax, tau[i]+dTau_max);
            d[i+n] = -std::max(tauMin, tau[i]-dTau_max);
        }
    }

    if(control == noMin)
        x = W.pseudoInverse() * w;
    else if(control == minT)     
        solve_qp::solveQP(Q, r, W, w, C, d, x, active);
    else if(control == minW)
        solve_qp::solveQPi(W, w, C, d, x, active);
    else if (control == cvxgen_minT)
    {
        cout << "cvxgen minimize tension"<<endl;
        int num_iters;
        // for the minimize tension
        for (int i = 0; i < 64; ++i)
        {
            if (i%9 == 0)
                params.Q[i] = 1.0;
            else
                params.Q[i] = 0.0; 
        }

        for (int i = 0; i < 8; ++i)
            params.c[i]=0;
        
        int k=0;
        // A matrix and b
        for (int j = 0; j < 8; ++j)
            for (int i = 0; i <6; ++i)
            {  
                params.A[k]=W[i][j];
                k++;
            }

        for (int i = 0; i < 6; ++i)
            params.b[i]=w[i];

        set_defaults();
        setup_indexing();
        // Solve problem instance for the record. 
        settings.verbose = 1;
        num_iters = solve();
        for (int i = 0; i < n; i++)
        {
            printf("  %9.4f\n", vars.x[i]);
            tau[i]=vars.x[i];
        }
        cout<<"difference"<< (W*tau-w).t()<<endl;
    }

    // slack variable by qp solver
    else if(control== slack_v)  
    {
        cout << "Using slack variable s" << endl;
        vpMatrix I_s;
        vpColVector tau_star(8), w_star(6);
        I_s.eye(6);
        // declare the targeted tension goal to be the minimum value
        for (int i = 0; i < 8; ++i)
            tau_star[i] = tauMin;

         w_star = W*tau_star;
        // establish the equality constraints
        A.insert(W,0,0);
        A.insert(I_s,0,8);
        b= w - w_star;
        // obtain tension through the qp solver 
        solve_qp::solveQP(Q, r, A, b, C, d, x, active);
        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // make up the realistic tension of taumin
        for (int i = 0; i < 8; ++i)
            x[i]+=tauMin;

        for (int i = 8; i < 14; ++i)
            cout << "slack variables" << x[i]<< ",";
    }

    // closed form
    else if( control == closed_form)
    {   
        // declaration 
        cout << "Using closed form" << endl;
        int num_r, index=0;
        vpColVector fm(n), tau_(n), w_(6);
        vpMatrix W_(6,n);
        double range_lim, norm_2;
        x= f_m + W.pseudoInverse() * (w - (W*f_m));
        f_v= x- f_m;
        fm=f_m;
        //compute the range limit of f_v
        norm_2 = sqrt(f_v.sumSquare());
        w_=w; W_=W ; num_r= n-6;
        //cout << "redundancy " << num_r<< endl;
        range_lim= sqrt(m)*(tauMax+tauMin)/4;
        //cout << "the maximal limit" << range_lim<<endl;
       for (int i = 0; i < n; ++i)
        {
            // judge the condition
            if ( norm_2 <= range_lim )
            {                 
                if (  (x.getMaxValue() > tauMax || x.getMinValue() < tauMin ) && num_r >0  )
                {
                    // search the relevant element which is unsatisfied
                    for (int j = 0; j < n; ++j)
                      {  
                        if(  x[j] == x.getMinValue() && x.getMinValue() < tauMin )
                                i = j;
                        else if  (x[j] == x.getMaxValue() && x.getMaxValue() > tauMax)
                                i=j;
                     }
                    cout << "previous tensions" << "  "<<x.t()<<endl;
                    cout << " i"<<"  "<< i <<endl;
                    // reduce the redundancy order
                    num_r--;
                    cout << "number of redundancy"<<"  "<< num_r<<endl;
                    // re- calculate the external wrench with maximal element
                    if ( x.getMaxValue() > tauMax)
                    {
                        w_= -tauMax*W_.getCol(i)+w_;
                        tau_[i]=tauMax;
                    }
                    else 
                    {
                        w_= - tauMin * W_.getCol(i)+w_;
                        tau_[i]=tauMin;
                    }
                    cout << "torque" << tau_.t() << endl;

                    fm[i]=0;

                    // drop relative column
                    W_[0][i]=W_[1][i]=W_[2][i]=W_[3][i]=W_[4][i]=W_[5][i]=0;

                    //compute the tensions again without unsatisfied component
                    x = fm + W_.pseudoInverse()*(w_- (W_*fm));

                    // construct the latest TD with particular components which equal to minimum and maximum
                    x=tau_+x;
                    cout << "tensions" << x.t() << endl;

                    // compute the force limit
                    f_v = x - f_m;
                    norm_2 = sqrt(f_v.sumSquare());
                    // initialize the index in order to inspect from the first electment
                    i = 0;
                }
                else if(num_r <0)
                    cout << "no feasible redundancy existing" << endl;
            }
            else
                cout << "no feasible tension distribution" << endl;
        }
    }

    else if ( control == Barycenter)
    {
        cout << "Using Barycenter" << endl;
        int inter=0;
        // compute the kernel of matrix W
        W.kernel(kerW);
        // obtain the particular solution of tensions
        p=W.pseudoInverse() * w;
        
        // lower and upper bound
        vpColVector A = -p, B = -p;
        for(int i=0;i<8;++i)
        {
            A[i] += tauMin;
            B[i] += tauMax;
        }

        // construct the projection matrix
        H = kerW.t();

        // build and publish H A B
        std_msgs::Float32MultiArray msg;
        msg.data.resize(32);
        for(int i=0; i<8 ;++i)
        {
            msg.data[4*i] = H[i][0];
            msg.data[4*i+1] = H[i][1];
            msg.data[4*i+2] = A[i];
            msg.data[4*i+3] = B[i];
        }
        bary_pub.publish(msg);

        // we look for points such as A <= H.x <= B

        // initialize vertices vector
        vertices.clear();

        // construct the 2x2 subsystem of linear equations in order to gain the intersection points in preimage
        for (int i = 0; i < n; ++i)
        {
            ker[0][0]=H[i][0];
            ker[0][1]=H[i][1];
            // eliminate the same combinations with initial value (i+1)
            for(int j=(i+1); j<n; ++j)
            {
                ker[1][0]=H[j][0];
                ker[1][1]=H[j][1];
                // pre-compute the inverse
                ker_inv = ker.inverseByLU();
                // the loop from A[i] to B[i];
                for(double u: {A[i],B[i]})
                {
                    for(double v: {A[j],B[j]})
                    {
                        // solve this intersection
                        F[0] = u;F[1] = v;
                        lambda = ker_inv * F;
                        inter++;
                        // check constraints, must take into account the certain threshold
                        if((H*lambda - A).getMinValue() >= - 1e-6 && (H*lambda - B).getMaxValue() <= 1e-6)
                             vertices.push_back(lambda);
                    }
                }
            }
        }
        cout << "the total amount of intersectioni points:" <<"  "<<inter << endl;
        // print the  satisfied vertices  number
        num_v = vertices.size();
        cout << "number of vertex:" << "  "<<num_v<< endl;
        for (int i = 0; i < vertices.size(); ++i)
           cout << "vertex " << "  "<< vertices[i].t()<<endl;

        vpColVector centroid(2);
        vpColVector ver(2), CoG(2);

        if( vertices.size() )
        {
            // compute centroid
            for(auto &vert: vertices)
                centroid += vert;
            centroid /= vertices.size();

            // compute actual CoG if more than 2 points
            if(vertices.size() > 2)
            {
                // re-order according to angle to centroid in clockwise order
                std::sort(vertices.begin(),vertices.end(),[&centroid](vpColVector v1, vpColVector v2)
                    {return atan2(v1[1]-centroid[1],v1[0]-centroid[0]) > atan2(v2[1]-centroid[1],v2[0]-centroid[0]);}); 

            /*                
                // compute CoG with trianglation alogorithm
                double a=0,v;
                centroid = 0; CoG=0; 
                for (int j= 1; j < (vertices.size()-1) ; ++j)
                {
                    ver+=(vertices[0]+vertices[j]+vertices[j+1]);
                    ver/=3;
                    v=(vertices[0][0]*vertices[j][1]+vertices[j][0]*vertices[j+1][1]+vertices[j+1][0]*vertices[0][1]
                        -vertices[0][0]*vertices[j+1][1]-vertices[j][0]*vertices[0][1]-vertices[j+1][0]*vertices[j][1])/2;
                    CoG+=ver*v;
                    a+=v;
                }
                centroid= CoG/a;
            */

           // compute CoG directly based on convex polygon
                vertices.push_back(vertices[0]);
                double a=0,v;
                centroid = 0;
                for(int i=1;i< vertices.size();++i)
                {
                    v = vertices[i-1][0]*vertices[i][1] - vertices[i][0]*vertices[i-1][1];
                    a += v;
                    centroid[0] += v*(vertices[i-1][0] + vertices[i][0]);
                    centroid[1] += v*(vertices[i-1][1] + vertices[i][1]);
                }
                centroid /= 3*a;
            }
            x = p+ H*centroid;
            cout << "the kernel "<< "  "<<(W*(H*centroid)).t()<<endl;
             //cout << "the residual "<< "  "<<(W*x-w).t()<<endl;
            cout << "the barycenter" << "  "<< centroid.t() << endl;
        }
        else 
            cout << "there is no vertex existing"<< endl;
    }

    //************************************************************************
    //* uncomment code below and header files for the slack variable algorithm 
    //************************************************************************
    /*else if ( control == cvxgen_slack)
    {
        cout<< "slack variable quadratic programming using CVXGEN "<<endl;
        vpColVector tau_star(8), w_star(6) ;
        int num_iters;

        // Make Q a diagonal PSD matrix, even though it's not diagonal. 
        // choose the targeted value to be the mean of tension limit
        for (int i = 0; i < 8; ++i)
            tau_star[i] = (tauMin+tauMax)/2;

        w_star = W*tau_star;
        // define the Q matrix (positive-semi-define)
        for (int i = 0; i < 196; i++)
        {   
            if (  i < 106)
            {   // D matrix for tau solution
                if ( i%15 == 0)
                    params.Q[i] = 1./(tauMax*tauMax);
                else
                    params.Q[i] = 0.0; 
            }
            else 
            {   // D matrix for slack variable
                if ( i%15 == 0)
                    params.Q[i] = 2.0;
                else
                    params.Q[i] = 0.0; 
            }   
        }
        // declare the C vector
        for (int i = 0; i < 14; ++i)   
            params.c[i]=0;
        // declare the A array, the entries are defined as the column order
        int k=0;
        // A matrix A=[ W I]  6x14  and b
        for (int j = 0; j < 14; ++j)
        { 
            for (int i = 0; i <6; ++i)
            {   
                if (j < 8)
                    params.A[k]=W[i][j];
                else if ( (j - i) == 8)
                    params.A[k] = 1.0;
                else 
                    params.A[k] = 0.0;
                k++;       
            }
        }
        // the right side of equality constraints 
        for (int i = 0; i < 6; ++i)
            params.b[i] = w[i] - w_star[i];

        set_defaults();
        setup_indexing();
        // Solve problem instance for the record. 
        settings.verbose = 1;
        num_iters = solve();
        for (int i = 0; i < n; i++)
        {
            printf("  %9.4f\n", vars.x[i]);
            x[i]=vars.x[i]+(tauMin+tauMax)/2;
        }            
    }
*/

    else
        cout << "No appropriate TDA " << endl;
   cout << "check constraints :" << endl;
            for(int i=0;i<n;++i)
                cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
    update_d = dTau_max;
    return tau;
}

vpColVector TDA::ComputeDistributionG(vpMatrix &W, vpColVector &ve, vpColVector &pe, vpColVector &w )
{   
    cout << " using variational gains algorithm based on quadratic problem" <<endl;

    //*********************************************************
    /*
    // save the matrices
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/Q_matrix", Q);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/A_matrix", A);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/r_matrix", r);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/b_matrix", b);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/d_matrix", d);
    */
    //**********************************************
    /*        
    std_msgs::Float32MultiArray msg;
    msg.data.resize(54);
    for(int i=0; i<6 ;++i)
    {
        msg.data[9*i] = W[i][0];
        msg.data[9*i+1] = W[i][1];
        msg.data[9*i+2] = W[i][2];
        msg.data[9*i+3] = W[i][3];
        msg.data[9*i+4] = W[i][4];
        msg.data[9*i+5] = W[i][5];
        msg.data[9*i+6] = W[i][6];
        msg.data[9*i+7] = W[i][7];
        msg.data[9*i+8] = w[i];
    }
    bary_pub.publish(msg);
    */
    //*********************************************
    /*    
    int num_iters;
    int Kp =80, Kd = 20;
    for (int i = 0; i < 144; i++)
    {   
        if (  i < 100) //(78 for 10 variables)
        {   // D matrix for tau solution
            if ( i%13 == 0)
                params.Q[i] = 1/(tauMin*tauMin);
            else
                params.Q[i] = 0.0; 
        }
        else 
        {    // D matrix for slack variable
            if ( i%13 == 0)
                params.Q[i] =1/25;
            else
                params.Q[i] = 0.0; 
        }   
    }
    // declare the C vector
    for (int i = 0; i < 12; ++i)   
        params.c[i]=0;
    // declare the A array, the entries are defined as the column order
    int k=0;
    // A matrix A=[ W -x -xd ]  6x10
    for (int j = 0; j < 12; ++j)
    { 
        for (int i = 0; i <6; ++i)
        {   
            if (j < 8)
                params.A[k] = W[i][j];
            else if ( j == 8 && i < 3)
                params.A[k] = -pe[i];
            else if ( j == 9 && i < 3)
                params.A[k] = -ve[i];
            else if ( j == 10 && i > 2)
                params.A[k] = -pe[i];
            else if ( j == 11 && i > 2)
                params.A[k] = -ve[i];
            else
                params.A[k] = 0;
            k++;     
        }
    }

    // the right side of equality constraints 
        for (int i = 0; i < 3; ++i)
            params.b[i] = w[i] + Kp*pe[i] + Kd*ve[i];
    
        for (int i = 3; i < 6; ++i)
            params.b[i] = w[i] + Kp*pe[i] + Kd*ve[i];

    set_defaults();
    setup_indexing();
    // solve problem instance for the record. 
    settings.verbose = 1;
    num_iters = solve();
    for (int i = 0; i < 12; i++)
    {
        printf("  %9.4f\n", vars.x[i]);
        x[i]=vars.x[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        w[i]+=(x[8]+Kp)*pe[i]+(x[9]+Kd)*ve[i];
        w[i+3]+=(x[10]+Kp)*pe[i+3]+(x[11]+Kd)*ve[i+3];
    }
    w_d = W*tau-w;
*/
    cout << "check constraints :" << endl;
    for(int i=0;i<n;++i)
        cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
    //update_d = dTau_max;
    return tau;
}

