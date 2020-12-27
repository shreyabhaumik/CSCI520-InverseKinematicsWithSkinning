#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

// comment this whole function to implement Dual Quaternion Skinning and make sure Dual Quaternion Skinning is uncommented
void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions, FK * fk, int SkinningMethod) const
{
  // Students should implement this

  // The following below is just a dummy implementation.
  // for(int i=0; i<numMeshVertices; i++)
  // {
  //   newMeshVertexPositions[3 * i + 0] = restMeshVertexPositions[3 * i + 0];
  //   newMeshVertexPositions[3 * i + 1] = restMeshVertexPositions[3 * i + 1];
  //   newMeshVertexPositions[3 * i + 2] = restMeshVertexPositions[3 * i + 2];
  // }

  /*---------------------------------------------Linear Blend Skinning(LBS)-------------------------------------------*/
  if(SkinningMethod == 0)
  {
    /* For every joint,
    jointSkinTransforms = (globalTransformOfJoint) * (globalNeutralTransformOfJoint)^(-1) // already done within FK while doing it
    For every vertex on the skin,
    newMeshVertexPositions = summationOverAllJoints(skinningWeightOfJoint * jointSkinTransform * restMeshVertexPositions)
    summationOverAllJoints(skinningWeightOfJoint) = 1, skinningWeightOfJoint >= 0
    */
    // for affine transformations multiplications we need 4D
    // to initialize newMeshVertexP for cumulative calculations
    std::vector<Vec4d> newMeshVertexP(numMeshVertices);
    for (int i = 0; i < numMeshVertices; i++)
      newMeshVertexP[i] = Vec4d(0.0, 0.0, 0.0, 0.0);
    // to populate newMeshVertexP with values from restMeshVertexPositions
    std::vector<Vec4d> restMeshVertexP(numMeshVertices);
    for (int i = 0; i < numMeshVertices; i++)
      restMeshVertexP[i] = Vec4d(restMeshVertexPositions[i * 3], restMeshVertexPositions[i * 3 + 1], restMeshVertexPositions[i * 3 + 2], 1.0);
    // calculating newMeshVertexPosition per vertex summing over all the joints influencing that vertex
    for(int i=0; i < numMeshVertices; i++)
      for(int j=0; j < numJointsInfluencingEachVertex; j++)
        newMeshVertexP[i] += meshSkinningWeights[i * numJointsInfluencingEachVertex + j] * jointSkinTransforms[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]] * restMeshVertexP[i];
    // populating new vertex positions for all the vertices
    for(int i=0; i < numMeshVertices; i++)
    {
      newMeshVertexPositions[i * 3] = newMeshVertexP[i][0];
      newMeshVertexPositions[i * 3 + 1] = newMeshVertexP[i][1];
      newMeshVertexPositions[i * 3 + 2] = newMeshVertexP[i][2];
    }
  }

  /*---------------------------------------------Dual Quaternion Skinning(DQS)------------------------------------------*/
  if(SkinningMethod == 1)
  {
    int n = fk->getNumJoints(); // total number of joints
    // translation for every joint
    std::vector<Vec3d> t(n);
    for(int i=0; i < n; i++)
    {
      Mat4d jST = jointSkinTransforms[i]; // a 4x4 row-major matrix
      t[i] = Vec3d(jST[0][3], jST[1][3], jST[2][3]);
    }
    // rotation matrix to quaternion for every joint 
    // coded using concept from pages G23, G24 of http://www.cs.cmu.edu/~baraff/pbm/rigid1.pdf
    std::vector<Vec4d> q0(n); // q0(w, i, j, k)
    for(int i=0; i < n; i++)
    {
      Mat4d jST = jointSkinTransforms[i]; // a 4x4 row-major matrix
      double R[9]= {jST[0][0], jST[0][1], jST[0][2], jST[1][0], jST[1][1], jST[1][2], jST[2][0], jST[2][1], jST[2][2]};
      double trace, u;
      trace = R[0] + R[4] + R[8];
      if(trace >= 0.0)
      {
        u = (double)sqrt(trace + 1.0);
        q0[i][0] = (double)0.5 * u;
        u = (double)0.5 / u;
        q0[i][1] = (R[7] - R[5]) * u;
        q0[i][2] = (R[2] - R[6]) * u;
        q0[i][3] = (R[3] - R[1]) * u;
      }
      else
      {
        int j = 0;
        if(R[4] > R[0])
          j = 1;

        if(R[8] > R[3*j+j])
          j = 2;

        switch (j)
        {
          case 0:
            u = (double)sqrt((R[0] - (R[4] + R[8])) + 1.0);
            q0[i][1] = (double)0.5 * u;
            u = (double)0.5 / u;
            q0[i][2] = (R[3] + R[1]) * u;
            q0[i][3] = (R[2] + R[6]) * u;
            q0[i][0] = (R[7] - R[5]) * u;
          break;

          case 1:
            u = (double)sqrt((R[4] - (R[8] + R[0])) + 1.0);
            q0[i][2] = (double)0.5 * u;
            u = (double)0.5 / u;
            q0[i][3] = (R[7] + R[5]) * u;
            q0[i][1] = (R[3] + R[1]) * u;
            q0[i][0] = (R[2] - R[6]) * u;
          break;

          case 2:
            u = (double)sqrt((R[8] - (R[0] + R[4])) + 1.0);
            q0[i][3] = (double)0.5 * u;
            u = (double)0.5 / u;
            q0[i][1] = (R[2] + R[6]) * u;
            q0[i][2] = (R[7] + R[5]) * u;
            q0[i][0] = (R[3] - R[1]) * u;
          break;
        }
      }
    }
    // using translation to compute quaternion q1 as: q1 = 0.5 * t * q0 for every joint
    std::vector<Vec4d> q1(n); // q1(w, i, j, k)
    for(int i=0; i < n; i++)
    {
      q1[i][0] = - (double)0.5 * ( t[i][0] * q0[i][1] + t[i][1] * q0[i][2] + t[i][2] * q0[i][3]);
      q1[i][1] = (double)0.5 * ( t[i][0] * q0[i][0] + t[i][1] * q0[i][3] - t[i][2] * q0[i][2]);
      q1[i][2] = (double)0.5 * (-t[i][0] * q0[i][3] + t[i][1] * q0[i][0] + t[i][2] * q0[i][1]);
      q1[i][3] = (double)0.5 * ( t[i][0] * q0[i][2] - t[i][1] * q0[i][1] + t[i][2] * q0[i][0]);
    }
    // after calculating t, we normalize q0 which will now be the rotation for every joint
    for(int i=0; i < n; i++)
      q0[i].normalize();
    // now for each vertex, we compute q0 and q1, compute tranformation matrix and newMeshVertexPositions
    for(int i=0; i < numMeshVertices; i++) // for each vertex
    {
      Vec4d vertexq0 = Vec4d(0.0, 0.0, 0.0, 0.0), vertexq1 = Vec4d(0.0, 0.0, 0.0, 0.0);
      Vec4d vertexq0conj, vertexq0inv, vertexT; // used to compute translation from vertexq0 and vertexq1
      Mat4d vertexSkinTransform = Mat4d(1.0); // in row-major order

      // computing q0 and q1
      for(int j=0; j < numJointsInfluencingEachVertex; j++)
      {
        int param = 1;
        // if(dot(q0[0], q0[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]]) < 0.0f)
        if(q0[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]][0] < 0) // to always choose the quaternion as close as possible
          param *= -1;
        vertexq0 += meshSkinningWeights[i * numJointsInfluencingEachVertex + j] * (param * q0[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]]);
        vertexq1 += meshSkinningWeights[i * numJointsInfluencingEachVertex + j] * (param * q1[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]]);
      }

      // computing tranformation matrix
      // computing translation of vertex from q0 and q1 as: t = 2 * q1 / q0
      double norm = len(vertexq0);  norm = (double)1.0 / norm;
      vertexq0 = vertexq0 * norm; // c0 = b0/||b0||
      vertexq1 = vertexq1 * norm; // cε = bε/||b0||
      vertexq0conj = Vec4d(vertexq0[0], -vertexq0[1], -vertexq0[2], -vertexq0[3]); // conjugate of vertexq0
      norm = len2(vertexq0);  norm = (double)1.0 / norm;
      vertexq0inv = vertexq0conj * norm; // inverse of vertexq0
      // (this is useless to compute)-> vertexT[0] = (double)2.0 * (vertexq1[0] * vertexq0inv[0] - vertexq1[1] * vertexq0inv[1] - vertexq1[2] * vertexq0inv[2] - vertexq1[3] * vertexq0inv[3]);
      vertexT[1] = (double)2.0 * (vertexq1[0] * vertexq0inv[1] + vertexq1[1] * vertexq0inv[0] + vertexq1[2] * vertexq0inv[3] - vertexq1[3] * vertexq0inv[2]);
      vertexT[2] = (double)2.0 * (vertexq1[0] * vertexq0inv[2] + vertexq1[2] * vertexq0inv[0] + vertexq1[3] * vertexq0inv[1] - vertexq1[1] * vertexq0inv[3]);
      vertexT[3] = (double)2.0 * (vertexq1[0] * vertexq0inv[3] + vertexq1[3] * vertexq0inv[0] + vertexq1[1] * vertexq0inv[2] - vertexq1[2] * vertexq0inv[1]);
      vertexSkinTransform[0][3] = vertexT[1];   vertexSkinTransform[1][3] = vertexT[2];   vertexSkinTransform[2][3] = vertexT[3];
      // computing rotation matrix from quaternion
      double W = vertexq0[0], X = vertexq0[1], Y = vertexq0[2], Z = vertexq0[3], two = 2.0;
      vertexSkinTransform[0][0] = 1.0 - two*Y*Y - two*Z*Z;  vertexSkinTransform[0][1] = two*X*Y - two*W*Z;      vertexSkinTransform[0][2] = two*X*Z + two*W*Y;
      vertexSkinTransform[1][0] = two*X*Y + two*W*Z;      vertexSkinTransform[1][1] = 1.0 - two*X*X - two*Z*Z;  vertexSkinTransform[1][2] = two*Y*Z - two*W*X;
      vertexSkinTransform[2][0] = two*X*Z - two*W*Y;      vertexSkinTransform[2][1] = two*Y*Z + two*W*X;      vertexSkinTransform[2][2] = 1.0 - two*X*X - two*Y*Y;

      // computing newMeshVertexPositions
      Vec4d newMeshVertexP, restMeshVertexP;
      newMeshVertexP = Vec4d(0.0, 0.0, 0.0, 0.0);
      restMeshVertexP = Vec4d(restMeshVertexPositions[i * 3], restMeshVertexPositions[i * 3 + 1], restMeshVertexPositions[i * 3 + 2], 1.0);
      newMeshVertexP = vertexSkinTransform * restMeshVertexP;
      newMeshVertexPositions[i * 3] = newMeshVertexP[0];
      newMeshVertexPositions[i * 3 + 1] = newMeshVertexP[1];
      newMeshVertexPositions[i * 3 + 2] = newMeshVertexP[2];
    }
  }
}
