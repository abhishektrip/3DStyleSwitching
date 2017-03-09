using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace DeformationTransfer.Utils
{
    public static class TriangleUtils
    {
        public static Vector3 CalculateFourthVertex(Vector3 v1, Vector3 v2, Vector3 v3)
        {
            // V = [v2-v1, v3-v1, v4-v1]    
            Vector3 v2_1 = v2 - v1;
            Vector3 v3_1 = v3 - v1;
            return CalculateFourthVertex(v2_1, v3_1);

        }
        public static Vector3 CalculateFourthVertex(Vector3 v2_1, Vector3 v3_1)
        {
            // Eq(1) : Fourth vertex in the direction perpendicular to the triangle. 
            // v4 = v1 + (v2-v1)x(v3-v1)/square root of |(v2-v1)x(v3-v1)| ( => sqrt of magnitude)
                  
           
            Vector3 v4_1 = new Vector3();

            v4_1 = Vector3.Cross(v2_1, v3_1);
            float norm = v4_1.magnitude;
            v4_1 = v4_1 /(float) Math.Sqrt(norm);

            return v4_1;
        }

        public static Matrix<float> CreateTriangleMatrix(Vector3 v1, Vector3 v2, Vector3 v3)
        {
            // V = [v2-v1, v3-v1, v4-v1]    
            Vector3 v2_1 = v2 - v1;
            Vector3 v3_1 = v3 - v1;
            Vector3 v4_1 = CalculateFourthVertex(v2_1, v3_1);

            float[,] mat = new float[3, 3] { { v2_1.x, v2_1.y, v2_1.z }, { v3_1.x, v3_1.y, v3_1.z }, { v4_1.x, v4_1.y, v4_1.z } };
            Matrix<float> triMatrix = Matrix<float>.Build.DenseOfArray(mat);

            return triMatrix;
        }
        public static Matrix<float> CalculateTransformationMatrix(Matrix<float> src, Matrix<float> dest)
        {
            // Q = Vdest * Inverse of Vsrc 
            var transform = src.Multiply(dest.Inverse());
            return transform;
        }
        public static Vector3 CalculateCoefficientV1(Matrix<float> invMat)
        {

            Vector3 CoeffV1 = new Vector3();

            CoeffV1.x = invMat[0, 0] + invMat[1, 0] + invMat[2, 0];
            CoeffV1.y = invMat[0, 1] + invMat[1, 1] + invMat[2, 1];
            CoeffV1.z = invMat[0, 2] + invMat[1, 2] + invMat[2, 2];

            return -1*CoeffV1;
        }
       
    }
}
