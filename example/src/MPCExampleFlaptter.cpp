/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the BSD 3-Clause License
 * @date 2018
 */


// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>

void setDynamicsMatrices(Eigen::Matrix<double, 2, 2> &a, Eigen::Matrix<double, 2, 1> &b)
{
    a << 1.,      0.020,
        0.,      0.9661;

    b << 0.,
        0.0315;
}


void setInequalityConstraints(Eigen::Matrix<double, 2, 1> &xMax, Eigen::Matrix<double, 2, 1> &xMin,
                              Eigen::Matrix<double, 1, 1> &uMax, Eigen::Matrix<double, 1, 1> &uMin)
{
    double u0 = 0.0;

    // input inequality constraints
    uMin << -6.0 - u0;

    uMax << 10.0 - u0;

    // state inequality constraints
    // TODO : change to present pos +/- ranges
    xMin << -100, -6.0;

    xMax << 100, 10.0;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 2> &Q, Eigen::DiagonalMatrix<double, 1> &R)
{
    Q.diagonal() << 2, 0;
    R.diagonal() << 0.2;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 2> &Q, const Eigen::DiagonalMatrix<double, 1> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix)
{

    hessianMatrix.resize(2*(mpcWindow+1) + 1 * mpcWindow, 2*(mpcWindow+1) + 1 * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<2*(mpcWindow+1) + 1 * mpcWindow; i++){
        if(i < 2*(mpcWindow+1)){
            int posQ=i%2;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%1;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 2> &Q, const Eigen::Matrix<double, 2, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double,2,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(2*(mpcWindow+1) +  1*mpcWindow, 1);
    for(int i = 0; i<2*(mpcWindow+1); i++){
        int posQ=i%2;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 2, 2> &dynamicMatrix, const Eigen::Matrix<double, 2, 1> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(2*(mpcWindow+1)  + 2*(mpcWindow+1) + 1 * mpcWindow, 2*(mpcWindow+1) + 1 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<2*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<2; j++)
            for(int k = 0; k<2; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(2 * (i+1) + j, 2 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 2; j++)
            for(int k = 0; k < 1; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(2*(i+1)+j, 1*i+k+2*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<2*(mpcWindow+1) + 1*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*2,i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 2, 1> &xMax, const Eigen::Matrix<double, 2, 1> &xMin,
                                   const Eigen::Matrix<double, 1, 1> &uMax, const Eigen::Matrix<double, 1, 1> &uMin,
                                   const Eigen::Matrix<double, 2, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(2*(mpcWindow+1) +  1 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(2*(mpcWindow+1) +  1 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(2*i,0,2,1) = xMin;
        upperInequality.block(2*i,0,2,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(1 * i + 2 * (mpcWindow + 1), 0, 1, 1) = uMin;
        upperInequality.block(1 * i + 2 * (mpcWindow + 1), 0, 1, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(2*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,2,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*2*(mpcWindow+1) +  1*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*2*(mpcWindow+1) +  1*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


void updateConstraintVectors(const Eigen::Matrix<double, 2, 1> &x0,
                             Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,2,1) = -x0;
    upperBound.block(0,0,2,1) = -x0;
}


double getErrorNorm(const Eigen::Matrix<double, 2, 1> &x,
                    const Eigen::Matrix<double, 2, 1> &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 2, 1> error = x - xRef;

    // return the norm
    return error.norm();
}


int main()
{
    // set the preview window
    int mpcWindow = 200;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 2, 1> xMax;
    Eigen::Matrix<double, 2, 1> xMin;
    Eigen::Matrix<double, 1, 1> uMax;
    Eigen::Matrix<double, 1, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 2> Q;
    Eigen::DiagonalMatrix<double, 1> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 2, 1> x0;
    Eigen::Matrix<double, 2, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // set the initial and the desired states
    x0 << 0, 0 ;
    xRef <<  1, 0;

    // set MPC problem quantities
    setDynamicsMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2 * (mpcWindow + 1) + 1 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 2 * (mpcWindow + 1) + 1 * mpcWindow);
    if(!solver.data()->setHessianMatrix(hessian)) return 1;
    if(!solver.data()->setGradient(gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if(!solver.data()->setLowerBound(lowerBound)) return 1;
    if(!solver.data()->setUpperBound(upperBound)) return 1;

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    // controller input and QPSolution vector
    Eigen::VectorXd ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    int numberOfSteps = 200;

    for (int i = 0; i < numberOfSteps; i++){

        // solve the QP problem
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(2 * (mpcWindow + 1), 0, 1, 1);

        // save data into file
        auto x0Data = x0.data();

        // propagate the model
        x0 = a * x0 + b * ctr;

        // update the constraint bound
        updateConstraintVectors(x0, lowerBound, upperBound);
        if(!solver.updateBounds(lowerBound, upperBound)) return 1;
      }
    return 0;
}
