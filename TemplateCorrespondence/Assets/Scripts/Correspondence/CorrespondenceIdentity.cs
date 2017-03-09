using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace DeformationTransfer.Correspondence
{
    public static class CorrespondenceIdentity
    {
        public static int CreateIdentityEquation(Matrix<float> A, Vector<float> C, CorrespondenceProblem problem, int row)
        {
            int matRow = row;
            int[] srcTriangles = problem.srcMesh.TriangleList;
            int numTris = problem.srcMesh.NumberOfTriangles;
            int triangleIndex = 0;
            var sqrtWt = Mathf.Sqrt(problem.wtIdentity);

            for (triangleIndex = 0; triangleIndex < numTris; triangleIndex++)
            {
                //Create Identity vector. 
                float[] cI = {1,0,0,
                              0,1,0,
                              0,0,1};
                Vector<float> cIdentity = Vector<float>.Build.DenseOfArray(cI);
                // T - I
                for(int iden = 0; iden<9; iden++)
                {
                    cIdentity[iden] += problem.srcMesh.matricesForTriangle[triangleIndex].C[iden];
                }
                // get A matrix for triangle. 
                var a = problem.srcMesh.matricesForTriangle[triangleIndex].A;
                // Add to linear system. 

                Utils.UtilityFunctions.AddToLinearEquationSystem(A, C, a, cIdentity, problem, triangleIndex, matRow, sqrtWt);                
                matRow += 9;
            }
            return matRow;
        }
    }
}
