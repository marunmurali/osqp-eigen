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
#include <fstream>

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
                        Eigen::SparseMatrix<double> &hessianMatrix, int Nx, int Nu)
{

    hessianMatrix.resize(Nx*(mpcWindow+1) + Nu * mpcWindow, Nx*(mpcWindow+1) + Nu * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<Nx*(mpcWindow+1) + Nu * mpcWindow; i++){
        if(i < Nx*(mpcWindow+1)){
            int posQ=i%Nx;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%Nu;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 2> &Q, const Eigen::Matrix<double, 2, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient, int Nx, int Nu)
{

    Eigen::Matrix<double,2,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(Nx*(mpcWindow+1) +  Nu*mpcWindow, 1);
    for(int i = 0; i<Nx*(mpcWindow+1); i++){
        int posQ=i%Nx;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 2, 2> &dynamicMatrix, const Eigen::Matrix<double, 2, 1> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix, int Nx, int Nu)
{
    constraintMatrix.resize(Nx*(mpcWindow+1)  + Nx*(mpcWindow+1) + Nu * mpcWindow, Nx*(mpcWindow+1) + Nu * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<Nx*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<Nx; j++)
            for(int k = 0; k<Nx; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(Nx * (i+1) + j, Nx * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < Nx; j++)
            for(int k = 0; k < Nu; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(Nx*(i+1)+j, Nu*i+k+Nx*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<Nx*(mpcWindow+1) + Nu*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*Nx,i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 2, 1> &xMax, const Eigen::Matrix<double, 2, 1> &xMin,
                                   const Eigen::Matrix<double, 1, 1> &uMax, const Eigen::Matrix<double, 1, 1> &uMin,
                                   const Eigen::Matrix<double, 2, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound, int Nx, int Nu)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(Nx*(mpcWindow+1) +  Nu * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(Nx*(mpcWindow+1) +  Nu * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(Nx*i,0,Nx,1) = xMin;
        upperInequality.block(Nx*i,0,Nx,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(Nu * i + Nx * (mpcWindow + 1), 0, Nu, 1) = uMin;
        upperInequality.block(Nu * i + Nx * (mpcWindow + 1), 0, Nu, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(Nx*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,Nx,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*Nx*(mpcWindow+1) +  Nu*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*Nx*(mpcWindow+1) +  Nu*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


void updateConstraintVectors(const Eigen::Matrix<double, 2, 1> &x0,
                             Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound, int Nx)
{
    lowerBound.block(0,0,Nx,1) = -x0;
    upperBound.block(0,0,Nx,1) = -x0;
}


double getErrorNorm(const Eigen::Matrix<double, 2, 1> &x,
                    const Eigen::Matrix<double, 2, 1> &xRef, int Nx)
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
    const int Nx = 2;
    const int Nu = 1;

    // allocate the dynamics matrices
    Eigen::Matrix<double, Nx, Nx> a;
    Eigen::Matrix<double, Nx, Nu> b;

    // allocate the constraints vector
    Eigen::Matrix<double, Nx, 1> xMax;
    Eigen::Matrix<double, Nx, 1> xMin;
    Eigen::Matrix<double, Nu, 1> uMax;
    Eigen::Matrix<double, Nu, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, Nx> Q;
    Eigen::DiagonalMatrix<double, Nu> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, Nx, 1> x0;
    Eigen::Matrix<double, Nx, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // set the initial and the desired states
    x0 << 0, 0 ;
    xRef <<  1/0.127, 0;

    // set MPC problem quantities
    setDynamicsMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian, Nx, Nu);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient, Nx, Nu);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix, Nx, Nu);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound, Nx, Nu);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(Nx * (mpcWindow + 1) + Nu * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * Nx * (mpcWindow + 1) + Nu * mpcWindow);
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

    std::ofstream myfile;
    myfile.open ("mpc_log.csv");
    myfile << "Log of MPC by osqp in C++.\n";
    myfile << "x1,x2,\n";

    for (int i = 0; i < numberOfSteps; i++){

        // solve the QP problem
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(Nx * (mpcWindow + 1), 0, Nu, 1);

        // save data into file
        auto x0Data = x0.data();

        // propagate the model
        x0 = a * x0 + b * ctr;
        myfile << x0[0] << "," << x0[1] <<",\n";


        // update the constraint bound
        updateConstraintVectors(x0, lowerBound, upperBound, Nx);
        if(!solver.updateBounds(lowerBound, upperBound)) return 1;
      }
    myfile.close();
    return 0;
}
