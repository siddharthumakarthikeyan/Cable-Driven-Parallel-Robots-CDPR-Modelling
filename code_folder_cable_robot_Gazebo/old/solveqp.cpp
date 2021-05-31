\

#include "ros/ros.h"
#include "cdpr3/SOLVEQP.h"

#include <iostream>
#include <cassert>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>


//Visp 
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpSubMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpThetaUVector.h>



// choose exact integral type

#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;


// program and solution types
typedef CGAL::Linear_program_from_iterators
<double**,                                                // for A
 double*,                                                 // for b
 CGAL::Const_oneset_iterator<CGAL::Comparison_result>, // for r
 bool*,                                                // for fl
 double*,                                                 // for l
 bool*,                                                // for fu
 double*,                                                 // for u
 //double**,                                                // for D
 double*>                                                 // for c 
Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

using namespace std;
bool CgalMethod(cdpr3::SOLVEQP::Request  &req, cdpr3::SOLVEQP::Response &res)
{

// We make use of qpOASES solver 

	/* Setup data of first QP. 
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 }; */


vpMatrix hessiann(8,8,0);
//hessiann.diag(1.0);
cout << "we are here 1" << endl;
//gradient vector
vpColVector grad(8, 1.0);
vpColVector fmin(8, 1.0);
vpColVector fmax(8, 300.0);
cout << "we are here 2" << endl;

 /*int  Ax[] = {1, -1};                        // column for x
  int  Ay[] = {1,  2};                        // column for y
  int*  A[] = {Ax, Ay};                       // A comes columnwise
  int   b[] = {7, 4};                         // right-hand side
                  // constraints are "<="
  bool fl[] = {true, true};                   // both x, y are lower-bounded
  int   l[] = {0, 0};
  bool fu[] = {false, true};                  // only y is upper-bounded
  int   u[] = {0, 4};                         // x's u-entry is ignored
  int  D1[] = {2};                            // 2D_{1,1}
  int  D2[] = {0, 8};                         // 2D_{2,1}, 2D_{2,2}
  int*  D[] = {D1, D2};                       // D-entries on/below diagonal
  int   c[] = {0, -32};
  double  c0   = 0;                             // constant term*/




double H1[] = {hessiann[0][0],hessiann[1][0],hessiann[2][0],hessiann[3][0],hessiann[4][0],hessiann[5][0],hessiann[6][0],hessiann[7][0] };
double H2[] = {hessiann[0][1],hessiann[1][1],hessiann[2][1],hessiann[3][1],hessiann[4][1],hessiann[5][1],hessiann[6][1],hessiann[7][1] } ;    
double H3[] = {hessiann[0][2],hessiann[1][2],hessiann[2][2],hessiann[3][2],hessiann[4][2],hessiann[5][2],hessiann[6][2],hessiann[7][2] } ;    
double H4[] = {hessiann[0][3],hessiann[1][3],hessiann[2][3],hessiann[3][3],hessiann[4][3],hessiann[5][3],hessiann[6][3],hessiann[7][3] };
double H5[] = {hessiann[0][4],hessiann[1][4],hessiann[2][4],hessiann[3][4],hessiann[4][4],hessiann[5][4],hessiann[6][4],hessiann[7][4] } ;    
double H6[] = {hessiann[0][5],hessiann[1][5],hessiann[2][5],hessiann[3][5],hessiann[4][5],hessiann[5][5],hessiann[6][5],hessiann[7][5] } ; 
double H7[] = {hessiann[0][6],hessiann[1][6],hessiann[2][6],hessiann[3][6],hessiann[4][6],hessiann[5][6],hessiann[6][6],hessiann[7][6] };
double H8[] = {hessiann[0][7],hessiann[1][7],hessiann[2][7],hessiann[3][7],hessiann[4][7],hessiann[5][7],hessiann[6][7],hessiann[7][7] } ;    

 double*  H[] = {H1, H2, H3, H4, H5, H6, H7, H8}; 

double A1[] = {req.WrenchMatrix[1],req.WrenchMatrix[9], req.WrenchMatrix[17]};
double A2[] = {req.WrenchMatrix[2],req.WrenchMatrix[10],req.WrenchMatrix[18]};   
double A3[] = {req.WrenchMatrix[3],req.WrenchMatrix[11],req.WrenchMatrix[19]};   
double A4[] = {req.WrenchMatrix[4],req.WrenchMatrix[12],req.WrenchMatrix[20]};
double A5[] = {req.WrenchMatrix[5],req.WrenchMatrix[13],req.WrenchMatrix[21]};
double A6[] = {req.WrenchMatrix[6],req.WrenchMatrix[14],req.WrenchMatrix[22]};
double A7[] = {req.WrenchMatrix[7],req.WrenchMatrix[15],req.WrenchMatrix[23]};
double A8[] = {req.WrenchMatrix[8],req.WrenchMatrix[16],req.WrenchMatrix[24]};   

double*  A[] = {A1, A2, A3, A4, A5, A6, A7, A8}; 

double   b[] = {req.wVector[0], req.wVector[1],req.wVector[2]}; 

bool fl[] = {true, true, true, true, true, true, true, true, true};
bool fu[] = {true, true, true, true, true, true, true, true, true};  

CGAL::Const_oneset_iterator<CGAL::Comparison_result> 
        r(    CGAL::EQUAL); 
   double  c0   = 0;  

double lb[] = {fmin[0],fmin[1],fmin[2],fmin[3],fmin[4],fmin[5],fmin[6],fmin[7]};
double ub[] = {fmax[0],fmax[1],fmax[2],fmax[3],fmax[4],fmax[5],fmax[6],fmax[7]};
double grad_vec[] = {grad[0],grad[1],grad[2],grad[3],grad[4],grad[5],grad[6],grad[7]};
 // now construct the quadratic program; the first two parameters are
  // the number of variables and the number of constraints (rows of A)
  //Program qp(8,3, A, b, r, fl, lb, fu, ub, H, grad_vec, c0);
   Program lp(8,3, A, b, r, fl, lb, fu, ub, grad_vec, c0);
   //solve the program, using ET as the exact type
  // print the program in MPS format

  //CGAL::print_quadratic_program(std::cout, qp, "first_qp");
  CGAL::print_linear_program(std::cout, lp, "first_lp");

 // Solution s = CGAL::solve_linear_program(qp, ET());
  //Solution SOL = CGAL::solve_quadratic_program(qp, ET());
  Solution SOL = CGAL::solve_linear_program(lp, ET());

  // output solution
  std::cout << SOL;

 

/*for (int i = 0; i < T.getCols(); i++)
{
cout << "This is xOPt " << s[i] << " This is the i " << i << endl;
}

cout << "we computed these forcev finally 3" <<endl;
forcev.resize(T.getCols(),true);
	for (int i = 0; i<(T.getCols()); i++)
	{ 
	forcev[i] = s[i];
	}

	//cout << "we computed these forcev finally" << forcev.t()<< endl;
	return forcev;
//delete H, A, grad_vec, lb, ub, lbA, ubA, xOpt;

	}else*/
res.tensions.clear();
//if (SOL.is_optimal())
//{ // we know that, don't we?
	    std::cout << "Basic constraints: ";
	    for (Solution::Index_iterator it = SOL.basic_constraint_indices_begin();
	     it != SOL.basic_constraint_indices_end(); ++it)
	      {res.tensions.push_back(*it);
	   std::cout << *it << "  ";
	    std::cout << std::endl;}
 //}else
//{
//for(int i=0;i<8;i++)
//{  res.tensions.push_back(fmin[i]);
//cout << "Pushing back tensions " << i << endl;}
//}



return true;
cout << "Could successfully return good forces calculated  " << endl;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "SOLVEQP");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("SOLVETheQP", CgalMethod);
  ROS_INFO("Ready to SOLVE QP.");
  ros::spin();

  return 0;
}




