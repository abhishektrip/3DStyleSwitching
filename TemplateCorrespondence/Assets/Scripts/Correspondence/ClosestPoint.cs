using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using DeformationTransfer.Utils;
using MathNet.Numerics.LinearAlgebra;

namespace DeformationTransfer.Correspondence
{
    public static class ClosestPoint
    {
        public static int[] CreateCorrespondenceMap_Basic(MeshData src, MeshData tgt)
        {
            int[] correspondenceMap = new int[src.NumberOfVertices];
            int closestVertIndex = -1;
            float distance = 0.0f, minDistance = float.MaxValue;

            for (int i = 0; i < src.NumberOfVertices; i++)
            {
                var srcVertex = src.VertexList[i];
                var srcNormal = src.NormalList[i];
                closestVertIndex = -1;
                for (int j = 0; j < tgt.NumberOfVertices; j++)
                {
                    var tgtVertex = tgt.VertexList[i];
                    var tgtNormal = tgt.NormalList[i];

                    if (Vector3.Dot(srcNormal, tgtNormal) > 0)
                    {
                        distance = UtilityFunctions.SquaredDistance(srcVertex, tgtVertex);
                        if (closestVertIndex == -1 || distance < minDistance)
                        {
                            minDistance = distance;
                            closestVertIndex = j;
                        }
                    }
                }
                correspondenceMap[i] = closestVertIndex;
            }
            return correspondenceMap;
        }
        public static int CreateClosestPointEquation(Matrix<float> A, Vector<float> C, int[] correspondenceMap, CorrespondenceProblem problem , int row, float wtClosest)
        {
            for (int i = 0; i < problem.srcMesh.NumberOfVertices; i++)
            {
                var vInfo = problem.vertexInfo.VertexInformation;
                if (vInfo[i].isConstrained == false)
                {
                    int iVx = problem.MatrixPositionForFreeVertex(i, 0);
                    int iVy = problem.MatrixPositionForFreeVertex(i, 1);
                    int iVz = problem.MatrixPositionForFreeVertex(i, 2);

                    int tgtIdx = correspondenceMap[i];

                    A[row, iVx] = wtClosest;
                    C[row] = wtClosest * problem.tgtMesh.VertexList[tgtIdx].x;
                    row++;

                    A[row, iVy] = wtClosest;
                    C[row] = wtClosest * problem.tgtMesh.VertexList[tgtIdx].y;
                    row++;

                    A[row, iVz] = wtClosest;
                    C[row] = wtClosest * problem.tgtMesh.VertexList[tgtIdx].z;
                    row++;
                }
            }
            return row;
        }
    }
}
