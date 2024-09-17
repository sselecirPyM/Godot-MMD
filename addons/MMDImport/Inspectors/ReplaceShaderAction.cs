using Godot;
using System.Collections.Generic;

namespace Mmd.addons.MMDImport.Inspectors
{
    public partial class ReplaceShaderAction : GodotObject
    {
        public Dictionary<Material, Material> Materials = new Dictionary<Material, Material>();
        public Dictionary<Material, Material> MaterialsReverse = new Dictionary<Material, Material>();

        public Node target;

        public void Do(Node3D target, Shader shader, EditorUndoRedoManager undoRedo)
        {
            this.target = target;
            CreateReplaceMaterials(shader);

            undoRedo.CreateAction("replace object shader materials", customContext: target);
            undoRedo.AddDoMethod(this, MethodName.DoReplaceAllMaterial);
            undoRedo.AddUndoMethod(this, MethodName.UndoReplaceAllMaterial);
            undoRedo.CommitAction();
        }

        void CreateReplaceMaterials(Shader shader)
        {
            foreach (var child in target.FindChildren("*", recursive: true))
            {
                if (child is not MeshInstance3D m1)
                {
                    continue;
                }
                for (int i = 0; i < m1.GetSurfaceOverrideMaterialCount(); i++)
                {
                    var material = m1.GetSurfaceOverrideMaterial(i) ?? m1.Mesh.SurfaceGetMaterial(i);
                    if (material != null)
                    {
                        if (Materials.TryGetValue(material, out var _))
                        {
                        }
                        else
                        {
                            var newMat1 = new ShaderMaterial();
                            newMat1.Shader = shader;
                            newMat1.RenderPriority = material.RenderPriority;
                            if (material is StandardMaterial3D standardMaterial)
                            {
                                newMat1.SetShaderParameter("Albedo", standardMaterial.AlbedoTexture);
                            }
                            else if (material is ShaderMaterial shaderMaterial1)
                            {
                                foreach (var parameter1 in shaderMaterial1.Shader.GetShaderUniformList())
                                {
                                    string name = ((string)parameter1.AsGodotDictionary()["name"]);
                                    newMat1.SetShaderParameter(name, shaderMaterial1.GetShaderParameter(name));
                                }
                            }

                            Materials.Add(material, newMat1);
                            MaterialsReverse.Add(newMat1, material);
                        }
                    }
                }
            }
        }


        void DoReplaceAllMaterial()
        {
            foreach (var child in target.FindChildren("*", recursive: true))
            {
                if (child is not MeshInstance3D m1)
                {
                    continue;
                }
                for (int i = 0; i < m1.GetSurfaceOverrideMaterialCount(); i++)
                {
                    var material = m1.GetSurfaceOverrideMaterial(i) ?? m1.Mesh.SurfaceGetMaterial(i);
                    if (material != null && Materials.TryGetValue(material, out var newMat))
                    {
                        m1.SetSurfaceOverrideMaterial(i, newMat);
                    }
                }
            }
        }
        void UndoReplaceAllMaterial()
        {
            foreach (var child in target.FindChildren("*", recursive: true))
            {
                if (child is not MeshInstance3D m1)
                {
                    continue;
                }
                for (int i = 0; i < m1.GetSurfaceOverrideMaterialCount(); i++)
                {
                    var material = m1.GetSurfaceOverrideMaterial(i) ?? m1.Mesh.SurfaceGetMaterial(i);
                    if (material != null && MaterialsReverse.TryGetValue(material, out var newMat))
                    {
                        m1.SetSurfaceOverrideMaterial(i, newMat);
                    }
                }
            }
        }
    }
}
