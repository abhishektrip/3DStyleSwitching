using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DeformationTransfer.Correspondence
{
    public struct VertexInfo
    {
        public bool isConstrained;
        public int MatrixPosition;
    }
    public class VertexPositionInfo
    {
        public VertexInfo[] VertexInformation;
        public int numberConstrained;
        public int numberFree;  

        public void PopulateVertexInformationList(CorrespondenceProblem problem)
        {
            int freeVertCount = 0;
            int constrainedVertCount = 0;            
            var srcMesh = problem.srcMesh;
            VertexInformation = new VertexInfo[srcMesh.VertexList.Length];
                        
            for (int i = 0; i < srcMesh.VertexList.Length; i++)
            {
                if(problem.vertexConstraintList.ContainsKey(i))
                {
                    //constrained.
                    VertexInformation[i] = new VertexInfo();
                    VertexInformation[i].isConstrained = true;
                    VertexInformation[i].MatrixPosition = constrainedVertCount;
                    constrainedVertCount++;                    
                }
                else
                {
                    //Free.
                    VertexInformation[i] = new VertexInfo();
                    VertexInformation[i].isConstrained = false;
                    VertexInformation[i].MatrixPosition = freeVertCount;
                    freeVertCount++;
                }
            }

            numberConstrained = constrainedVertCount;
            numberFree = freeVertCount;
        }
    }
}
