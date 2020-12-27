#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.

  int n = fk.getNumJoints(); // total number of joints // to save computation everytime
  // using vectors of Mat3<real> and Vec3<real> for passing as values to multiplyAffineTransform4ds
  std::vector<Mat3<real>> localTransforms(n), globalTransforms(n); // for the transformations
  std::vector<Vec3<real>> localTranslations(n), globalTranslations(n); // for the translations

  // to compute localTransforms and localTranslations
  for(int i = 0; i < n; i++)
  {
    Mat3<real> eulerRotMat, jointOEulerMat; // to store intermediate matrices
    real angle[3];
    // eulerAngles
    angle[0] = eulerAngles[i * 3];
    angle[1] = eulerAngles[i * 3 + 1];
    angle[2] = eulerAngles[i * 3 + 2];
    eulerRotMat = Euler2Rotation(angle, fk.getJointRotateOrder(i));
    // jointOrientationEulerAngles
    Vec3d jOrient = fk.getJointOrient(i);
    angle[0] = jOrient.data()[0];
    angle[1] = jOrient.data()[1];
    angle[2] = jOrient.data()[2];
    jointOEulerMat = Euler2Rotation(angle, XYZ);
    // localTransform
    localTransforms[i] = jointOEulerMat * eulerRotMat;
    // translations
    globalTranslations[i][0] = localTranslations[i][0] = fk.getJointRestTranslation(i)[0]; // the global translations here are temp, they will be calculated later
    globalTranslations[i][1] = localTranslations[i][1] = fk.getJointRestTranslation(i)[1];
    globalTranslations[i][2] = localTranslations[i][2] = fk.getJointRestTranslation(i)[2];
  }

  // to compute globalTransforms and globalTranslations
  for(int i = 0; i < n; i++)
  {
    int current = fk.getJointUpdateOrder(i); // Get the joint that appears at position "i" in a linear joint update order.
    int parentOfCurrent = fk.getJointParent(current);

    if(parentOfCurrent == -1) // means current is root
      globalTransforms[current] = localTransforms[current];
    else
      multiplyAffineTransform4ds(globalTransforms[parentOfCurrent], globalTranslations[parentOfCurrent], localTransforms[current], localTranslations[current], globalTransforms[current], globalTranslations[current]);
  }

  // to compute the handle positions
  // std::cout << n <<' '<< numIKJoints <<"\n";
  for(int i = 0; i < numIKJoints; i++) // numIKJoints is number of IK handles
  { // The position of a handle is the global translation of the joint.
    int jID = IKJointIDs[i];
    handlePositions[i * 3] = globalTranslations[jID][0];
    handlePositions[i * 3 + 1] = globalTranslations[jID][1];
    handlePositions[i * 3 + 2] = globalTranslations[jID][2];
  }
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp.
  
  // following the exact format in ADOLCExample.cpp:
  int n = FKInputDim; // input dimension is n // n is (3 × the number of joints)
  int m = FKOutputDim; // output dimension is m // m is (3 × the number of IK handles)

  trace_on(adolc_tagID); // start tracking computation with ADOL-C // getting the tag value from "this->adolc_tagID = adolc_tagID;" in IK::IK

  vector<adouble> eAngles(n); // define the input of the function f
  for(int i = 0; i < n; i++)
    eAngles[i] <<= 0.0; // The <<= syntax tells ADOL-C that these are the input variables.

  vector<adouble> hPositions(m); // define the output of the function f

  // The computation of f goes here:
  forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eAngles, hPositions); // using forwardKinematicsFunction for computation of f

  vector<double> output(m);
  for(int i = 0; i < m; i++)
    hPositions[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables

  // Finally, call trace_off to stop recording the function f.
  trace_off(); // ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles, int IKalgo)
{
  // You may find the following helpful:
  int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.

  // following the exact format in ADOLCExample.cpp:

  int n = FKInputDim; // input dimension is n // n is (3 × the number of joints)
  int m = FKOutputDim; // output dimension is m // m is (3 × the number of IK handles)
  /* Will use vectors of double for everything for ease of transition from calculations in adol to eigen */
  // now, you can call ::function(adolc_tagID, ...) as many times as you like to ask ADOL-C to evaluate f for different x:
  // double input_x_values[] = {0.0, 0.1, 0.2}; - this will be 'jointEulerAngles->data()'
  std::vector<double> hPositions(m); // this will be double output_y_values[] = {0.0, 0.0};
  ::function(adolc_tagID, m, n, jointEulerAngles->data(), hPositions.data());

  // You can call ::jacobian(adolc_tagID, ...) as many times as you like to ask ADOL-C to evalute the jacobian matrix of f on different x:
  std::vector<double> jacobianMatrix(m*n); // We store the matrix in row-major order.
  std::vector<double *> jacobianMatrixEachRow(m); // pointer array where each pointer points to one row of the jacobian matrix
  for(int i = 0; i < m; i++)
    jacobianMatrixEachRow[i] = &jacobianMatrix[n*i];
  ::jacobian(adolc_tagID, m, n, jointEulerAngles->data(), jacobianMatrixEachRow.data()); // each row is the gradient of one output component of the function

  std::vector<double> deltaTheta(n); // deltaTheta is a n x 1 vector representing the change of Euler angles we want to find
  Eigen::VectorXd x(n); // intermediate to calculate values of deltaTheta

  /*---------------------------------------------Tikhonov regularization---------------------------------------------*/
  /*
  (J^T*J + α*I)deltaTheta = (J^T*deltab)
  i.e. A x = b 
        where     A is (J^T*J + α*I),
                  x is deltaTheta
                  b is (J^T*deltab)
  Also,
  deltaTheta is a n x 1 vector representing the change of Euler angles we want to find
  deltab is a m × 1 vector representing the change of handle global positions
  */
  if(IKalgo == 0)
  {
    // step 1 - (COMPUTING A) Need to get J, J^T, I to compute (J^T*J + α*I) i.e A
    // J
    Eigen::MatrixXd J(m,n); // m rows and n columns column major matrix
    for(int rowID = 0; rowID < m; rowID++)
      for(int colID = 0; colID < n; colID++)
        J(rowID,colID) = jacobianMatrix[n*rowID+colID];
    // J^T
    Eigen::MatrixXd JT(n,m);
    JT = J.transpose();
    // I
    Eigen::MatrixXd I(n,n);
    I = Eigen::MatrixXd::Identity(n,n);
    // A i.e. (J^T*J + α*I)
    double α = 0.01; // α determines how much regularization to add
    Eigen::MatrixXd A(n,n);
    A = JT * J + α * I;
    // step 2 - (COMPUTING b) Need to get deltab to compute (J^T*deltab) i.e b
    vector<double> rhs(m);
    Eigen::VectorXd deltab(m);
    for(int i = 0; i < m; i++)
    {
      rhs[i] = targetHandlePositions->data()[i] - hPositions.data()[i];
      deltab[i] = rhs[i];
    }
    Eigen::VectorXd b(m);
    b = JT * deltab;
    // step 3 - (COMPUTING x) now solve for x in A x = b
    // note: here, we assume that A is symmetric; hence we can use the LDLT decomposition
    x = A.ldlt().solve(b);
  }

  /*---------------------------------------------Pseudoinverse method-----------------------------------------------*/
  /*
    deltaTheta = J† * deltaP
      where     J† is J^T(J*J^T)^-1

  Also,
  deltaTheta is a n x 1 vector representing the change of Euler angles we want to find
  deltaP is a m × 1 vector representing the change of handle global positions
  */
  if(IKalgo == 1)
  {
    // step 1 - (COMPUTING J†) i.e. J^T(J*J^T)^-1
    //J
    Eigen::MatrixXd J(m,n); // m rows and n columns column major matrix
    for(int rowID = 0; rowID < m; rowID++)
      for(int colID = 0; colID < n; colID++)
        J(rowID,colID) = jacobianMatrix[n*rowID+colID];
    // J^T
    Eigen::MatrixXd JT(n,m);
    JT = J.transpose();
    // (J*J^T)
    Eigen::MatrixXd JJT(m,m);
    JJT =  J * JT;
    // (J*J^T)^-1
    Eigen::MatrixXd JJTinv(m,m);
    JJTinv = JJT.inverse();
    Eigen::MatrixXd JTJJTinv(n,m);
    // J^T(J*J^T)^-1 
    JTJJTinv = JT * JJTinv;
    // step 2 - (COMPUTING J† * deltaP) 
    vector<double> rhs(m);
    Eigen::VectorXd deltaP(m);
    for(int i = 0; i < m; i++)
    {
      rhs[i] = targetHandlePositions->data()[i] - hPositions.data()[i];
      deltaP[i] = rhs[i];
    }
    x = JTJJTinv * deltaP;
  }

  /*---------------------------------------------Transpose method-----------------------------------------------*/
  /*
    deltaTheta = α * J^T * deltaP

  Also,
  deltaTheta is a n x 1 vector representing the change of Euler angles we want to find
  deltaP is a m × 1 vector representing the change of handle global positions
  */
  if(IKalgo == 2)
  {
    //J
    Eigen::MatrixXd J(m,n); // m rows and n columns column major matrix
    for(int rowID = 0; rowID < m; rowID++)
      for(int colID = 0; colID < n; colID++)
        J(rowID,colID) = jacobianMatrix[n*rowID+colID];
    // J^T
    Eigen::MatrixXd JT(n,m);
    JT = J.transpose();
    // J^T * deltaP
    vector<double> rhs(m);
    Eigen::VectorXd deltaP(m);
    for(int i = 0; i < m; i++)
    {
      rhs[i] = targetHandlePositions->data()[i] - hPositions.data()[i];
      deltaP[i] = rhs[i];
    }
    x = JT * deltaP;
    // α * J^T * deltaP
    double α = deltaP.norm(); // choosing α is very critical here, it needs to be chosen as close as possible to deltaP
    x = α * x;
  }

  // to update jointEulerAngles
  for(int i = 0; i < n; i++)
    deltaTheta[i] = x[i];
  Vec3d dTheta;
  for(int i = 0; i < numJoints; i++)
  {
    dTheta = Vec3d(deltaTheta[i * 3], deltaTheta[i * 3 + 1], deltaTheta[i * 3 + 2]);
    jointEulerAngles[i] += dTheta;
  }
}
