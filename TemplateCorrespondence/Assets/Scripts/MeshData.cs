using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using DeformationTransfer.Utils;

namespace DeformationTransfer.Correspondence
{
    public struct MeshMatrixData
    {
        public Matrix<float> A;
        public Vector<float> C; 
    }
    public class MeshData
    {
        struct matrixSize
        {
            int rows, cols;
        }

        //public SkinnedMeshRenderer smr;

        #region Mesh data
        int[] triangleList;
        Vector3[] vertexList;
        Vector3[] normalList;
        #endregion
        HashSet<int> constrainedVertexList = new HashSet<int>();
        public List<Matrix<float>> VertexMatrices = new List<Matrix<float>>();
        public List<Matrix<float>> InverseVertexMatrices = new List<Matrix<float>>();
        public List<MeshMatrixData> matricesForTriangle = new List<MeshMatrixData>();
        public Vector3[] FourthVertexForTriangle; 

        public int NumberOfTriangles
        {
            get { return triangleList.Length/3; }
        }
        public int[] TriangleList
        {
            get { return triangleList; }
            set { triangleList = value; }
        }
        public Vector3[] VertexList
        {
            get { return vertexList; }
            set { vertexList = value; }
        }
        public int NumberOfVertices
        {
            get { return vertexList.Length; }
        }
        public Vector3[] NormalList
        {
            get { return normalList; }
            set { normalList = value; }
        }
        public int NumberOfNormals
        {
            get { return normalList.Length; }
        }

        public void ComputeVertexMatricesForTriangles()
        {
            var triangles = TriangleList;
            var vertices = VertexList;
            FourthVertexForTriangle = new Vector3[NumberOfTriangles];

            for (int i =0; i< NumberOfTriangles; i++ )
            {
                int v1 = triangles[i * 3 + 0];
                int v2 = triangles[i * 3 + 1];
                int v3 = triangles[i * 3 + 2];

                FourthVertexForTriangle[i] = TriangleUtils.CalculateFourthVertex(vertices[v1], vertices[v2], vertices[v3]);

                var vertexMat = TriangleUtils.CreateTriangleMatrix(vertices[v1], vertices[v2], vertices[v3]);

                VertexMatrices.Add(vertexMat);
                InverseVertexMatrices.Add(vertexMat.Inverse());

                Debug.Log("Matrix for Tri #" + i + " : " + vertexMat.ToString());
                Debug.Log("Inverse Matrix for Tri #" + i + " : " + InverseVertexMatrices[i].ToString());
            }        
        }
        
    }
}
