#ifndef QP_H
#define QP_H

#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <algorithm>

namespace solve_qp
{

using std::cout;
using std::endl;
using std::vector;

/* Solves a quadratic minimization under equality constraint with projection approach
 * min_x ||Q.x - r||^2
 * st. A.x = b
 */
void solveQPe ( const vpMatrix &_Q, const vpColVector _r, const vpMatrix &_A, const vpColVector &_b, vpColVector &_x)
{
    vpMatrix _Ap = _A.pseudoInverse();
    vpColVector x1 = _Ap * _b;
    vpMatrix P;P.eye(x1.getRows());
    P -= _Ap * _A;
    _x = (_Q*P).pseudoInverse() * (_r - _Q*x1);
}



/* Solves a quadratic minimization under equality and inequality constraint, uses projection
 * min_x ||Q.x - r||^2
 * st. A.x = b
 * st. C.x <= d
 */
void solveQP ( const vpMatrix &_Q, const vpColVector _r, vpMatrix _A, vpColVector _b, const vpMatrix &_C, const vpColVector &_d, vpColVector &_x, std::vector<bool> &active)
{
    // check data coherence
    const unsigned int n = _Q.getCols();
    if (    n != _A.getCols() ||
            n != _C.getCols() ||
            _A.getRows() != _b.getRows() ||
            _C.getRows() != _d.getRows() ||
            _Q.getRows() != _r.getRows())
    {
        cout << "solveQP: wrong dimension" << endl <<
                "Q: " << _Q.getRows() << "x" << _Q.getCols() << " - r: " << _r.getRows() << endl <<
                "A: " << _A.getRows() << "x" << _A.getCols() << " - b: " << _b.getRows() << endl <<
                "C: " << _C.getRows() << "x" << _C.getCols() << " - d: " << _d.getRows() << endl;
        return;
    }


    unsigned int i,j;
    const unsigned int nA = _A.getRows();
    const unsigned int nC = _C.getRows();

    if(active.size() != nC)
        active.resize(nC, false);

    // look for trivial solution (case of only inequalities, often the highest priority for robot constraints)
    // if the constraints are satisfied without moving, just do not move
    // r = 0, b empty or null, d > 0 -> x = 0
    if(_r.euclideanNorm() == 0 &&
            (_d.getRows() == 0 || _d.getMinValue() >= 0) &&
            (_b.getRows() == 0 || _b.euclideanNorm() == 0))
    {
        //cout << "okSolveQP::solveQP: trivial solution"	 << endl;
        _x.resize(n);
        return;
    }

    // no trivial solution, go for solver
    vector< vector<bool> > activePast;
    vector<bool> activeBest = active;
    activePast.reserve(5);
    unsigned int nAct = count ( active.begin(), active.end(),true );

    double ineqMax, errCur, errBest = -1;
    unsigned int ineqInd, ineqCount;
    vpMatrix In;In.eye(n);
    vpMatrix P = In;
    vpMatrix Ap;
    vpColVector x(n), l, e;
    vpSubMatrix ApC;

    // solve at one iteration
    while ( true )
    {
        activePast.push_back ( active );

        // update A and b with activated inequality constraints
        _A.resize(nA + nAct, n, false );
        _b.resize ( nA + nAct, false );

        // active set from C and d
        ineqCount = 0;
        for ( i=0;i<nC;++i )
        {
            if ( active[i] )
            {
                for ( j=0;j<n;++j )
                {
                    _A[nA+ineqCount][j] = _C[i][j];
                    _b[nA+ineqCount] = _d[i];

                }
                ineqCount++;
            }
        }
        // end init A and b

        // solve with projection if any constraint matrix
        if(_A.getRows())
        {
            // print what we solve
          /*  cout << "A " << endl << _A <<endl;
            cout << "b " << endl << _b.t() <<endl;
            cout << "Q " << endl << _Q <<endl;
            cout << "r " << endl << _r.t() <<endl;
            cout << "C " << endl << _C <<endl;
            cout << "d " << endl << _d.t() <<endl;*/

            Ap = _A.pseudoInverse();
            x = Ap * _b;
            P = In - Ap * _A;
            // check values of P
            for(i=0;i<n;++i)
                for(j=0;j<n;++j)
                    if(P[i][j] < 1e-6)
                        P[i][j] = 0;
          //  cout << "P" << endl << P << endl;
            x += P*(_Q*P).pseudoInverse() * (_r - _Q*x);  
            // check for infeasible program
            if(_A.getRows() >= n && P.getMaxValue() - P.getMinValue() < 1e-6)
            {   // full rank constraints
                vpColVector cons = _A * x - _b;
                //cout<< "the difference in QP:"<<"    "<< cons << endl;
                if(cons.getMaxValue() - cons.getMinValue() > 1e-6)
                {
                    cout << "--------------------------------------------------QP seems infeasible----------------------------------------------------\n";
                    x.resize(0);
                    return;
                }
            }
        }
        else
            x = _Q.pseudoInverse()*_r;

        //cout << "temporary solution: " << x.t() << endl;

        // find strongest violated inequality in Cx > d
        ineqMax = 0;
        for ( i=0;i<nC;++i )
        {
            if ( !active[i] &&  _C.getRow ( i ) * x - _d[i] > ineqMax + 1e-6 )
            {
                ineqMax = _C.getRow ( i ) * x - _d[i];
                ineqInd = i;
                //cout << "ineqMax for " << i << ": " << ineqMax << endl;
            }
        }

        if ( ineqMax != 0 )			// active worst violated equality
        {
            nAct++;
            active[ineqInd] = true;
            //cout << "activating inequality # " << ineqInd << endl;
        }
        else						// all inequalities ensured, ineqMax==0
        {
            // this solution is feasible, store it if it is the best found up to now
            e = (_Q*x - _r);
            errCur = e.euclideanNorm();
            if ( errBest == -1 || errCur < errBest )
            {
                errBest = errCur;
                activeBest = active;
                _x = x;
            }

            // try to deactivate a constraint, compute Lagrange multiplier
            ineqMax = 0;
            if(nAct)
            {
                ApC.init(Ap, 0, nA, n, nAct);
                l = -ApC.transpose() * _Q.transpose() * e;

                ineqCount = 0;

                for ( i=0;i<nC;++i )
                    if ( active[i] )
                    {
                        if ( l[ineqCount] < ineqMax )
                        {
                            ineqMax = l[ineqCount];
                            ineqInd = i;
                        }
                        ineqCount++;
                    }
            }
            // deactive most useless inequality if any
            if ( ineqMax != 0 )
            {
                active[ineqInd] = false;
                nAct--;
                //cout << "deactivating inequality # " << ineqInd << endl;
            }
            else	// no useless equality, this has to be the optimal solution
                break;
        }

        // before looping again, check whether the new active set candidate has already been tested or not
        ineqInd = 0;
        ineqCount = 0;
        for ( auto const &prev: activePast)
        {
            ineqCount = 0;
            for ( j=0;j<nC;++j )
            {
                if ( prev[j] == active[j] )
                    ineqCount++;
            }
            // if nC same values: new active set has already been tested, we're beginning a cycle
            // leave the loop
            if ( ineqCount == nC )
                return;
        }
    }
    active = activeBest;
}


/* Solves a quadratic minimization under inequality constraint
 * Just a particular case of general problem
 * min_x ||Q.x - r||^2
 * st. C.x <= d
 */
void solveQPi ( const vpMatrix &Q, const vpColVector r, vpMatrix C, const vpColVector &d, vpColVector &x, std::vector<bool> &active)
{
    vpMatrix A ( 0,Q.getCols() );
    vpColVector b ( 0 );
    solveQP ( Q, r, A, b, C, d, x, active);
}

}


#endif
