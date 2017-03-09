using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using DeformationTransfer.Utils;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearRegression;

namespace DeformationTransfer.Correspondence
{
    public class TriangleAdjacency
    {
        public int adjacentTriangleCount;
        public HashSet<int>[] adjacencyList; 
    }
    public class CorrespondenceProblem
    {
        #region Data containers
        public MeshData srcMesh, tgtMesh; // meshes to create correspondence
        public TriangleAdjacency triAdj; //= new HashSet<int>[];
        public Dictionary<int, int> vertexConstraintList = new Dictionary<int, int>();
        public VertexPositionInfo vertexInfo = new VertexPositionInfo();
        #endregion

        #region Weights
        public float wtSmoothness;
        public float wtIdentity;
        public float wtClosestStart, wtClosestEnd, wtClosestStep;
        #endregion

        #region Result        
        public CorrespondenceResult result;
        #endregion

        #region Methods
        public void Solve()
        {
            Debug.Log("Computing the vertex information");
            vertexInfo.PopulateVertexInformationList(this);
            Debug.Log("Computing Vertex Martices");
            srcMesh.ComputeVertexMatricesForTriangles();
            Debug.Log("Computing A & C Matrices ");
            PopulateMatricesForTriangles();

            SolvePhase1();
            SolvePhase2();

        }
        public int BuildPhase1Equation(Matrix<float> A, Vector<float> C)
        {
            int rowIdx = 0;
            Debug.Log(" Adding Smoothness equation ..");
            rowIdx = CorrespondenceSmoothness.CreateSmoothnessEquation(A, C, this, rowIdx);
            Debug.Log(String.Format("Row Index : {0}", rowIdx));
            Debug.Log(" Adding Identity equation...");
            rowIdx = CorrespondenceIdentity.CreateIdentityEquation(A, C, this, rowIdx);
            Debug.Log(String.Format("Row Index : {0}", rowIdx));
            return rowIdx;
        }
        public void SolvePhase1()
        {
            int nRow = 9 * (triAdj.adjacentTriangleCount + srcMesh.NumberOfTriangles);
            int nCol = 3 * (vertexInfo.numberFree + srcMesh.NumberOfTriangles);

            Debug.Log(String.Format("Rows : {0}  \n Cols : {1}", nRow, nCol));
            Matrix<float> A = Matrix<float>.Build.Sparse(nRow, nCol, 0);
            Vector<float> C = Vector<float>.Build.Dense(nRow, 1);

            BuildPhase1Equation(A, C);

            Debug.Log("Linear System Solve...");

            Debug.Log("A : " + A.ToString());
            Debug.Log("C : " + C.ToString());

            // compute the QR decomposition
            //var qr = A.QR();

            //// get orthogonal matrix with first n columns of Q
            //Matrix<float> Q1 = qr.Q.SubMatrix(0, nRow, 0, nCol);
            //Debug.Log("Q1 : " + Q1.ToString());
            //// get upper-triangular matrix of size n x n
            //Matrix<float> R = qr.R.SubMatrix(0, nCol, 0, nCol);
            //Debug.Log("R : " + R.ToString());
            //var x = R.Inverse().Multiply(Q1.Transpose().Multiply(C));


            var x = A.Transpose().Multiply(A).Inverse().Multiply(A.Transpose().Multiply(C));
            //var x = A.Transpose().Multiply(A).Cholesky().Solve(A.Transpose().Multiply(C));
            //var x = A.Solve(C);

            Debug.Log("x : " + x.ToString());

            ApplyDeformationToSourceMesh(x);
        }
        public int BuildPhase2Equation(Matrix<float> A, Vector<float> C, int[] correspondenceMap, float wtClosestPoint)
        {
            int rowIdx = 0;

            rowIdx = BuildPhase1Equation(A, C);
            rowIdx = ClosestPoint.CreateClosestPointEquation(A, C, correspondenceMap, this, rowIdx, wtClosestPoint);

            return rowIdx;
        }
        public void SolvePhase2()
        {
            Debug.Log("Phase 2 Starting...");

            int nRow = 9 * (triAdj.adjacentTriangleCount + srcMesh.NumberOfTriangles) + 3 * (vertexInfo.numberFree);
            int nCol = 3 * (vertexInfo.numberFree + srcMesh.NumberOfTriangles);

            Debug.Log(String.Format("Rows : {0}  \n Cols : {1}", nRow, nCol));
            Matrix<float> A;
            Vector<float> C;
            for (float wt = wtClosestStart; wt < wtClosestEnd; wt += wtClosestStep)
            {
                Debug.Log("Iteration Step " + wt);
                Debug.Log("Creating Correspondence map..");
                var correspondenceMap = ClosestPoint.CreateCorrespondenceMap_Basic(srcMesh, tgtMesh);

                A = Matrix<float>.Build.Sparse(nRow, nCol, 0);
                C = Vector<float>.Build.Dense(nRow, 1);

                Debug.Log("Building Equations");
                BuildPhase2Equation(A, C, correspondenceMap, wt);

                Debug.Log("Linear System Solve...");

                //Debug.Log("A : " + A.ToString());
                //Debug.Log("C : " + C.ToString());

                var x = A.Transpose().Multiply(A).Cholesky().Solve(A.Transpose().Multiply(C));

                Debug.Log("Applying Deformation....");
                ApplyDeformationToSourceMesh(x);
            }

        }
        public void CalculateTriangleMatrices( int index )
        {

            var A = srcMesh.matricesForTriangle[index].A;
            var C = srcMesh.matricesForTriangle[index].C;

            var inverseMat = srcMesh.InverseVertexMatrices[index];
            var coefv1 = TriangleUtils.CalculateCoefficientV1(srcMesh.InverseVertexMatrices[index]);
            float coef = 0;

            int v1 = index * 3 + 0;
            int v2 = index * 3 + 1;
            int v3 = index * 3 + 2;

            for (int i = 0, row = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++, row++)
                {
                    C[row] = 0;

                    /* coefficient of v1: -(a[0,i] + a[1,i] + a[2,i]) */
                    coef = coefv1[j];

                    if (!vertexConstraintList.ContainsKey(srcMesh.TriangleList[v1]))
                    {
                        A[row, 0] = coef;
                    }
                    else
                    {
                        var tgtVertIdx = vertexConstraintList[srcMesh.TriangleList[v1]];
                        C[row] -= coef * tgtMesh.VertexList[tgtVertIdx][i];
                    }
                    

                    /* coefficient of v2: a[0,i]*/
                    coef = inverseMat[0,j];
                    if (!vertexConstraintList.ContainsKey(srcMesh.TriangleList[v2]))
                    {
                        A[row, 1] = coef;
                    }
                    else
                    {
                        var tgtVertIdx = vertexConstraintList[srcMesh.TriangleList[v2]];
                        C[row] -= coef * tgtMesh.VertexList[tgtVertIdx][i];
                    }

                    /* coefficient of v3: a[1,i] */
                    coef = inverseMat[1,j];
                    if (!vertexConstraintList.ContainsKey(srcMesh.TriangleList[v3]))
                    {
                        A[row, 2] = coef;
                    }
                    else
                    {
                        var tgtVertIdx = vertexConstraintList[srcMesh.TriangleList[v3]];
                        C[row] -= coef * tgtMesh.VertexList[tgtVertIdx][i];
                    }

                    /* coefficient of v4: a[2,i] */
                    A[row,3] = inverseMat[2,j];
                }
            }
        }
        public void PopulateMatricesForTriangles()
        {
            
            int triIndex = 0; 

            for (triIndex = 0; triIndex < srcMesh.NumberOfTriangles; triIndex++)
            {
                MeshMatrixData mdata = new MeshMatrixData();
                mdata.A = Matrix<float>.Build.Sparse(9, 4);
                mdata.C = Vector<float>.Build.Dense(9);

                srcMesh.matricesForTriangle.Add(mdata);
                

                CalculateTriangleMatrices(triIndex);

                if (display)
                    DisplayMatrices(triIndex);               
            }

        }
        public int MatrixPositionForTriangle(int triIdx, int vertIdx, int componentIdx)
        {
            int vIdx = 0;
            int position = -1;
            if (vertIdx < 3)
            {
                vIdx = srcMesh.TriangleList[triIdx * 3 + vertIdx];
                position = MatrixPositionForFreeVertex(vIdx, componentIdx);
                //Debug.Log(String.Format("Vert index : {0} -> position : {1}", vIdx, position));
            }
            else if(vertIdx == 3)
            {
                position = MatrixPositionForFourthVertex(triIdx, vertIdx, componentIdx);
            }
            return position;
        }
        public int MatrixPositionForFreeVertex(int vertIdx, int componentIdx)
        {
            int position = -1;
            if (!vertexInfo.VertexInformation[vertIdx].isConstrained)
                position = vertexInfo.VertexInformation[vertIdx].MatrixPosition * 3 + componentIdx;

            return position;
        }
        public int MatrixPositionForFourthVertex(int triIdx, int vertIdx, int componentIdx)
        {
            int position = -1;
            if (vertIdx == 3)
            {
                position = (vertexInfo.numberFree + triIdx) * 3 + componentIdx;
            }
            return position;
        }
        public void ApplyDeformationToSourceMesh(Vector<float> deformedX)
        {
            for(int i = 0; i < srcMesh.NumberOfVertices; i++)
            {
                if(vertexInfo.VertexInformation[i].isConstrained)
                {
                    int tgtIdx = vertexConstraintList[i];
                    srcMesh.VertexList[i] = tgtMesh.VertexList[tgtIdx];
                }
                else
                {
                    Vector3 deformedVert = new Vector3();

                    deformedVert.x = deformedX[MatrixPositionForFreeVertex(i, 0)];
                    deformedVert.y = deformedX[MatrixPositionForFreeVertex(i, 1)];
                    deformedVert.z = deformedX[MatrixPositionForFreeVertex(i, 2)];

                    srcMesh.VertexList[i] = deformedVert;
                }
            }
        }
        #endregion

        #region Debug Methods
        bool display = true;
        void DisplayMatrices(int index)
        {
            var A = srcMesh.matricesForTriangle[index].A;
            var C = srcMesh.matricesForTriangle[index].C;

            Debug.Log(" Matrix A for Tri Idx [ " + index + "] : " + A.ToString());
            Debug.Log(" Vector C for Tri Idx [ " + index + "] : " + C.ToString());
        }
        public void DisplayAdjacenecyList()
        {
            int triIndex = 0;
            foreach (var adjTriList in triAdj.adjacencyList)
            {
                foreach (var tri in adjTriList)
                    Debug.Log(triIndex + " --> " + tri);
                triIndex++;
            }
        }
        #endregion

    }

    public class CorrespondenceResultContainer
    {
        public int srcIndex;
        public int tgtIndex;
        public float distance;
    }
    //Serializable ?
    public class CorrespondenceResult
    {
        public List<CorrespondenceResultContainer> correspondenceList = new List<CorrespondenceResultContainer>();
    }
}
