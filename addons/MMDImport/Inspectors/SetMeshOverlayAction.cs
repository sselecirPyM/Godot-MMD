using Godot;
using System.Collections.Generic;

namespace Mmd.addons.MMDImport.Inspectors
{
    public partial class SetMeshOverlayAction : GodotObject
    {
        public Dictionary<Node, Material> Materials = new Dictionary<Node, Material>();
        public Dictionary<Node, Material> MaterialsReverse = new Dictionary<Node, Material>();

        public Node target;
        public Material materialToReplace;

        public void Do(Node3D target, Material materialToReplace, EditorUndoRedoManager undoRedo)
        {
            this.target = target;
            this.materialToReplace = materialToReplace;
            CreateReplaceMaterials();

            undoRedo.CreateAction("set mesh overlay materials", customContext: target);
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
                Materials.Add(child, materialToReplace);
                MaterialsReverse.Add(child, m1.MaterialOverlay);
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
                if (Materials.TryGetValue(m1, out var material))
                {
                    m1.MaterialOverlay = material;
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
                if (MaterialsReverse.TryGetValue(m1, out var material))
                {
                    m1.MaterialOverlay = material;
                }
            }
        }
    }
}
