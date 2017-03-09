using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace DeformationTransfer.Correspondence
{
    public static class CorrespondenceSmoothness
    {
        public static int CreateSmoothnessEquation(Matrix<float> A, Vector<float> C,CorrespondenceProblem problem, int row)
        {
            int matRow = row;
            int[] srcTriangles = problem.srcMesh.TriangleList;
            int numTris = problem.srcMesh.NumberOfTriangles;
            int triangleIndex = 0;
            var sqrtWt = Mathf.Sqrt(problem.wtSmoothness);
            for(triangleIndex = 0; triangleIndex < numTris; triangleIndex++)
            {
                var adjTriangles = problem.triAdj.adjacencyList[triangleIndex];
                foreach(var adjTriangle  in adjTriangles)
                {
                    //Debug.Log("Triangle Pair #1 :" + triangleIndex);
                    Utils.UtilityFunctions.AddToLinearEquationSystem(A, C, problem, triangleIndex, matRow, sqrtWt);                   
                    //Debug.Log("Triangle Pair #2 :" + adjTriangle);
                    Utils.UtilityFunctions.AddToLinearEquationSystem(A, C, problem, adjTriangle, matRow, -sqrtWt);
                    matRow += 9;
                }
            }
            return matRow;
        }
    }
}
