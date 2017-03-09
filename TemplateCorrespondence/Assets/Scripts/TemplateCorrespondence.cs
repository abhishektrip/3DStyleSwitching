using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using DeformationTransfer.Utils;

namespace DeformationTransfer.Correspondence
{
    public class TemplateCorrespondence : MonoBehaviour
    {
        public GameObject srcGO;
        public GameObject tgtGO;
        //public SkinnedMeshRenderer TargetMesh;
        public CorrespondenceProblem Problem; 
        //public Material mat; 
        Vector3 c0, c1, c2;
        //Camera main;

        void Start()
        {
            //camera = GetComponent<Camera>();
            // Source & target mesh . 
            Vector3[] srcVerts = new Vector3[] { new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(2, 1, 0), new Vector3(2, 0, 0), new Vector3(3, 1, 0) };
            Vector3[] tgtVerts = new Vector3[] { new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(2, 1, 0), new Vector3(2, 0, 0), new Vector3(3, 0.5f, 0) };

            int[] tris = new int[] {0,1,3,
                                       1,2,3,
                                       2,4,3};

            //GameObject srcGO = new GameObject("Source GO");
            //GameObject tgtGO = new GameObject("Target GO");
            //srcGO.transform.SetParent(gameObject.transform);
            //tgtGO.transform.SetParent(gameObject.transform);

            var srcMF = srcGO.AddComponent<MeshFilter>();
            var srcMR = srcGO.AddComponent<MeshRenderer>();
            srcMR.material = new Material(Shader.Find("Diffuse"));

            var tgtMF = tgtGO.AddComponent<MeshFilter>();
            var tgtMR = tgtGO.AddComponent<MeshRenderer>();
            tgtMR.material = new Material(Shader.Find("Diffuse"));


            Mesh srcMesh = new Mesh();
            srcMesh.vertices = srcVerts;
            srcMesh.triangles = tris;
            srcMF.mesh = srcMesh;

            Mesh tgtMesh = new Mesh();
            tgtMesh.vertices = tgtVerts;
            tgtMesh.triangles = tris;
            tgtMF.mesh = tgtMesh;

            srcMF.mesh.RecalculateNormals();
            tgtMF.mesh.RecalculateNormals();

            srcGO.GetComponent<MeshCollider>().sharedMesh = srcMF.mesh;
            tgtGO.GetComponent<MeshCollider>().sharedMesh = tgtMF.mesh;            

            ResolveCorrespondence();

        }
        //void OnPostRender()
        //{
        //    if (!mat)
        //    {
        //        Debug.LogError("Please Assign a material on the inspector");
        //        return;
        //    }
        //    GL.PushMatrix();
        //    mat.SetPass(0);
        //    //GL.LoadOrtho();
        //    GL.Color(Color.red);
        //    GL.Begin(GL.TRIANGLES);
        //    //GL.Color(Color.red);
        //    GL.Vertex3(c0[0], c0[1], c0[2]);
        //    GL.Vertex3(c1[0], c1[1], c1[2]);
        //    GL.Vertex3(c2[0], c2[1], c2[2]);
        //    GL.End();
        //    GL.PopMatrix();
        //}

        void Update()
        {
            RaycastHit hit;
            var cameraAttached = GetComponent<Camera>();
            if (!Physics.Raycast(cameraAttached.ScreenPointToRay(Input.mousePosition), out hit))
                return;

            MeshCollider meshCollider = hit.collider as MeshCollider;
            if (meshCollider == null || meshCollider.sharedMesh == null)
                return;

            Mesh mesh = meshCollider.sharedMesh;
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            Vector3 p0 = vertices[triangles[hit.triangleIndex * 3 + 0]];
            Vector3 p1 = vertices[triangles[hit.triangleIndex * 3 + 1]];
            Vector3 p2 = vertices[triangles[hit.triangleIndex * 3 + 2]];
            Transform hitTransform = hit.collider.transform;
            c0 = hitTransform.TransformPoint(p0);
            c1 = hitTransform.TransformPoint(p1);
            c2 = hitTransform.TransformPoint(p2);

            Debug.DrawLine(c0, c1);//, Color.green);
            Debug.DrawLine(c1, c2);//, Color.green);
            Debug.DrawLine(c2, c0);//, Color.green);

            var lineRend = GetComponent<LineRenderer>();
            if (lineRend != null)
            {
                lineRend.SetPosition(0, c0);
                lineRend.SetPosition(1, c1);
                lineRend.SetPosition(2, c2);
            }
            //GenerateCorrespondence();
        }

        public void AddTriangleMarkersToList()
        {

        }
        public void GenerateCorrespondence()
        {
            // Add Correspondence code here.
            Matrix<double> A = DenseMatrix.OfArray(new double[,] {
        {1,1,1,1},
        {1,2,3,4},
        {4,3,2,1}});
            Vector<double>[] nullspace = A.Kernel();

            // verify: the following should be approximately (0,0,0)
            var x = (A * (2 * nullspace[0] - 3 * nullspace[1]));
        }
        public void ResolveCorrespondence()
        {
            Problem = new CorrespondenceProblem();

            // Source & target mesh . 
            //Vector3[] srcVerts = new Vector3[] { new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(2, 1, 0), new Vector3(2, 0, 0), new Vector3(3, 1, 0) };
            //Vector3[] tgtVerts = new Vector3[] { new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(2, 1, 0), new Vector3(2, 0, 0), new Vector3(3, 0.5f, 0) };

            //int[] tris = new int[] {0,1,3,
            //                        1,2,3,
            //                        2,4,3};

            var srcMesh = srcGO.GetComponent<MeshFilter>().sharedMesh;
            var tgtMesh = tgtGO.GetComponent<MeshFilter>().sharedMesh;
            Problem.srcMesh = new MeshData();
            Problem.tgtMesh = new MeshData();

            Problem.srcMesh.VertexList = srcMesh.vertices;  //srcVerts; // = this.SourceMesh;
            Problem.srcMesh.TriangleList = srcMesh.triangles; //tris;
            Problem.srcMesh.NormalList = srcMesh.normals;


            Problem.tgtMesh.VertexList = tgtMesh.vertices; //tgtVerts;     //= this.TargetMesh;
            Problem.tgtMesh.TriangleList = tgtMesh.triangles; //tris;
            Problem.tgtMesh.NormalList = tgtMesh.normals;

            // Vertex Constraint List
            Problem.vertexConstraintList = new Dictionary<int, int>();
            Problem.vertexConstraintList.Add(0, 0);
            Problem.vertexConstraintList.Add(1, 1);
            Problem.vertexConstraintList.Add(4, 4);

            Problem.wtSmoothness = 1.0f;
            Problem.wtIdentity = 0.01f;

            Problem.wtClosestStart = 1.0f;
            Problem.wtClosestEnd = 5000.0f;
            Problem.wtClosestStep = 1250f;


            // Create Adjacency list for source model. 
            Problem.triAdj = UtilityFunctions.CreateAdjacencyList(Problem.srcMesh.NumberOfVertices, Problem.srcMesh.TriangleList);

            Problem.Solve();
        }
    }
}