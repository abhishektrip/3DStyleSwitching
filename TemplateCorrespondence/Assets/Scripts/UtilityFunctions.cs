using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using DeformationTransfer.Correspondence;

namespace DeformationTransfer.Utils
{
    public static class UtilityFunctions
    {
        public static float SquaredDistance(Vector3 a, Vector3 b)
        {
            float x = a.x - b.x;
            float y = a.y - b.y;
            float z = a.z - b.z;

            return x * x + y * y + z * z;
        }
        // Set True if constraint , false if not. 
        //public static bool[] CreateVertexConstraintInfo(Dictionary<int,int>, )
        public static TriangleAdjacency CreateAdjacencyList(int numVertices, int[] triangles)
        {
            HashSet<int>[] vertMembershipList = new HashSet<int>[numVertices];
            HashSet<int>[] adjList = new HashSet<int>[triangles.Length/3];
            int triangleIndex = 0;
            int i = 0;

            //initialize arrays
            for (i = 0; i < adjList.Length; i++)
                adjList[i] = new HashSet<int>();
            for (i = 0; i < vertMembershipList.Length; i++)
                vertMembershipList[i] = new HashSet<int>();

            // Prepare data for faster triangle adjacency list creation. 
            for (i = 0; i < triangles.Length; i += 3)
            {
                int v1i = i;
                int v2i = i+1;
                int v3i = i+2;

                //Check if index is within bounds.                          
                vertMembershipList[triangles[v1i]].Add(triangleIndex);
                vertMembershipList[triangles[v2i]].Add(triangleIndex);
                vertMembershipList[triangles[v3i]].Add(triangleIndex);
                triangleIndex ++;
            }
            triangleIndex = 0;
            int totalAdjTriangles = 0; 
            // find adjacency
            for (i = 0; i < triangles.Length; i += 3)
            {
                int v1i = i;
                int v2i = i + 1;
                int v3i = i + 2;
               
                List<IEnumerable<int>> adjTriangleIndexList = new List<IEnumerable<int>>();
                var v1v2 = vertMembershipList[triangles[v1i]].Intersect(vertMembershipList[triangles[v2i]]);
                var v1v3 = vertMembershipList[triangles[v1i]].Intersect(vertMembershipList[triangles[v3i]]);
                var v2v3 = vertMembershipList[triangles[v2i]].Intersect(vertMembershipList[triangles[v3i]]);

                adjTriangleIndexList.Add(v1v2);
                adjTriangleIndexList.Add(v1v3);
                adjTriangleIndexList.Add(v2v3);

                for (int j = 0; j < adjTriangleIndexList.Count; j++)
                {
                    var triangleForEdge = adjTriangleIndexList[j];
                    if (triangleForEdge.Count() >  1)
                    {
                        //max two indices should be available
                        //one edge can only be shared between two triangles, hence max two. 
                        totalAdjTriangles++;
                        adjList[triangleForEdge.ElementAt(0)].Add(triangleForEdge.ElementAt(1));
                        adjList[triangleForEdge.ElementAt(1)].Add(triangleForEdge.ElementAt(0));                   
                    }
                }
            }

            return new TriangleAdjacency { adjacentTriangleCount = totalAdjTriangles, adjacencyList = adjList };
        }
        public static void AddToLinearEquationSystem(Matrix<float> A, Vector<float> C, CorrespondenceProblem problem, int triIdx, int row, float weight)
        {
            int iRow = row;
            int jRow = 0;
            for (int compIdx = 0; compIdx < 3; compIdx++)
            {
                for (int iter = 0; iter < 3; iter++, iRow++, jRow++)
                {
                    for (int vert = 0; vert < 4; vert++)
                    {
                        var position = problem.MatrixPositionForTriangle(triIdx, vert, compIdx);
                        if (position != -1)
                        {
                            A[iRow, position] = weight * problem.srcMesh.matricesForTriangle[triIdx].A[jRow, vert];
                        }
                    }
                    //Debug.Log("Current Row i : " + iRow + "\n Current Row j : " + jRow);
                    C[iRow] = C[iRow] + weight * problem.srcMesh.matricesForTriangle[triIdx].C[jRow];
                }
            }
        }
        public static void AddToLinearEquationSystem(Matrix<float> A, Vector<float> C, Matrix<float> a, Vector<float> c, CorrespondenceProblem problem, int triIdx, int row, float weight)
        {
            int iRow = row;
            int jRow = 0;
            for (int compIdx = 0; compIdx < 3; compIdx++)
            {
                for (int iter = 0; iter < 3; iter++, iRow++, jRow++)
                {
                    for (int vert = 0; vert < 4; vert++)
                    {
                        var position = problem.MatrixPositionForTriangle(triIdx, vert, compIdx);
                        if (position != -1)
                        {
                            //Debug.Log("A[" + iRow + " , " + jRow +"]");
                            A[iRow, position] = weight * a[jRow, vert];
                        }
                    }
                    //Debug.Log("Current Row i : " + iRow + "\n Current Row j : " + jRow);
                    C[iRow] = C[iRow] + weight * c[jRow];
                }
            }
        }
    }
}
