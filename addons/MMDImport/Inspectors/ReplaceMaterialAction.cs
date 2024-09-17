using Godot;
using System.Collections.Generic;

namespace Mmd.addons.MMDImport.Inspectors
{
    public partial class ReplaceMaterialAction : GodotObject
    {
        public Dictionary<Material, Material> Materials = new Dictionary<Material, Material>();
        public Dictionary<Material, Material> MaterialsReverse = new Dictionary<Material, Material>();

        public Node target;

        public void DuplicateMaterials(Node target, EditorUndoRedoManager undoRedo)
        {
            this.target = target;
            CreateReplaceMaterials();

            undoRedo.CreateAction("duplicate object materials", customContext: target);
            undoRedo.AddDoMethod(this, MethodName.DoReplaceAllMaterial);
            undoRedo.AddUndoMethod(this, MethodName.UndoReplaceAllMaterial);
            undoRedo.CommitAction();
        }

        void CreateReplaceMaterials()
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

                    if (material != null && material is ShaderMaterial shaderMaterial)
                    {
                        if (Materials.TryGetValue(shaderMaterial, out var newMaterial))
                        {
                        }
                        else
                        {
                            newMaterial = (ShaderMaterial)shaderMaterial.Duplicate(true);

                            Materials.Add(material, newMaterial);
                            MaterialsReverse.Add(newMaterial, material);
                        }
                    }
                }
            }
        }

        public void ReplaceMaterialWithPreset(Node target, Material source, Material toMaterial, EditorUndoRedoManager undoRedo)
        {
            this.target = target;
            CreateReplaceMaterials2(source, toMaterial);

            undoRedo.CreateAction("replace material with preset", customContext: target);
            undoRedo.AddDoMethod(this, MethodName.DoReplaceAllMaterial);
            undoRedo.AddUndoMethod(this, MethodName.UndoReplaceAllMaterial);
            undoRedo.CommitAction();
        }

        void CreateReplaceMaterials2(Material source, Material toMaterial)
        {
            Materials.Add(source, toMaterial);
            MaterialsReverse.Add(toMaterial, source);
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
