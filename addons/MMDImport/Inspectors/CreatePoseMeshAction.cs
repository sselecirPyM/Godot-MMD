using Godot;
using System.Collections.Generic;

namespace Mmd.addons.MMDImport.Inspectors
{
    public partial class CreatePoseMeshAction : EditorInspectorPlugin
    {
        public Node target;

        MeshInstance3D generatedMeshInstance;

        public void Do(Node3D target, EditorUndoRedoManager undoRedo)
        {
            this.target = target;
            CreateModels();

            undoRedo.CreateAction("create pose mesh", customContext: target);
            undoRedo.AddDoMethod(this, MethodName.DoAttachNode);
            undoRedo.AddUndoMethod(this, MethodName.DoDetachNode);

            undoRedo.CommitAction();
        }

        public void CreateModels()
        {
            Skeleton3D skeleton = null;
            Transform3D[] trans = null;
            List<Vector3> position = new List<Vector3>();
            List<Vector3> normal = new List<Vector3>();
            List<int> indice = new List<int>();
            foreach (var child in target.FindChildren("*"))
            {
                if (child is Skeleton3D s3)
                {
                    skeleton = s3;
                    trans = new Transform3D[s3.GetBoneCount()];
                    for (int i = 0; i < s3.GetBoneCount(); i++)
                    {
                        //GD.Print($"p1{skeleton.GetBoneGlobalRest(i).Origin},p2{skeleton.GetBoneGlobalPose(i).Origin}");
                        trans[i] = skeleton.GetBoneGlobalPose(i) * skeleton.GetBoneGlobalRest(i).Inverse();
                    }
                }
                else if (child is MeshInstance3D meshInstance1)
                {
                    if (meshInstance1.Skin == null)
                    {
                        continue;
                    }
                    ProcessMesh(position, normal, indice, trans, meshInstance1);

                }
            }
            var mesh1 = new ArrayMesh();
            Godot.Collections.Array surfaceArray = new Godot.Collections.Array();
            surfaceArray.Resize((int)ArrayMesh.ArrayType.Max);
            surfaceArray[(int)Mesh.ArrayType.Vertex] = position.ToArray();
            surfaceArray[(int)Mesh.ArrayType.Normal] = normal.ToArray();
            surfaceArray[(int)Mesh.ArrayType.Index] = indice.ToArray();
            mesh1.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, surfaceArray);
            generatedMeshInstance = new MeshInstance3D();
            generatedMeshInstance.Mesh = mesh1;
            GD.Print("create pose 3");
        }

        void ProcessMesh(List<Vector3> position, List<Vector3> normal, List<int> indice, Transform3D[] trans, MeshInstance3D meshInstance1)
        {
            var mesh = (ArrayMesh)meshInstance1.Mesh;
            for (int i = 0; i < mesh.GetSurfaceCount(); i++)
            {
                var arrays = mesh.SurfaceGetArrays(i);

                int baseVert = position.Count;

                var p1 = arrays[(int)Mesh.ArrayType.Vertex].As<Godot.Collections.Array<Vector3>>();
                var n1 = arrays[(int)Mesh.ArrayType.Normal].As<Godot.Collections.Array<Vector3>>();
                var b1 = arrays[(int)Mesh.ArrayType.Bones].As<Godot.Collections.Array<int>>();
                var w1 = arrays[(int)Mesh.ArrayType.Weights].As<Godot.Collections.Array<float>>();
                for (int j = 0; j < mesh.GetBlendShapeCount(); j++)
                {
                    var weight = meshInstance1.GetBlendShapeValue(j);
                    if (Mathf.Abs(weight) < 1e-4)
                        continue;
                    var arr1 = mesh.SurfaceGetBlendShapeArrays(j);
                    var bs1 = arr1[0];
                    for (int k = 0; k < bs1.Count; k++)
                    {
                        p1[k] += bs1[k].AsVector3() * weight;
                    }
                }
                for (int j = 0; j < p1.Count; j++)
                {
                    Vector3 p2 = Vector3.Zero;
                    Vector3 n2 = Vector3.Zero;
                    for (int k = 0; k < 4; k++)
                    {
                        int boneIndex = b1[j * 4 + k];
                        float weight = w1[j * 4 + k];
                        var t = trans[boneIndex];
                        p2 += t * p1[j] * weight;
                        n2 += t.Basis * n1[j] * weight;
                    }
                    n2 = n2.Normalized();

                    p1[j] = p2;
                    n1[j] = n2;
                }

                position.AddRange(p1);
                normal.AddRange(n1);

                var index1 = arrays[(int)Mesh.ArrayType.Index].As<Godot.Collections.Array<int>>();
                for (int j = 0; j < index1.Count; j++)
                {
                    index1[j] += baseVert;
                }
                indice.AddRange(index1);
            }
        }

        public void DoAttachNode()
        {
            var par = target.GetParent();
            var rt = par.GetTree().EditedSceneRoot;
            generatedMeshInstance.Name = target.Name;
            par.AddChild(generatedMeshInstance);
            generatedMeshInstance.Owner = rt;
        }

        public void DoDetachNode()
        {
            var par = target.GetParent();
            par.RemoveChild(generatedMeshInstance);
        }
    }
}
