using Godot;

namespace Mmd.addons.MMDImport.Inspectors
{
    [Tool]
    public partial class MMDInspectorPlugin : EditorInspectorPlugin
    {

        MMDModel currentModel;
        FileDialog fileDialog;


        public override bool _CanHandle(GodotObject @object)
        {
            if (@object is Node3D nd)
            {
                if (nd is MMDModel md)
                {
                    currentModel = md;
                }
                else
                {
                    currentModel = null;
                }
                return true;
            }
            return false;
        }

        public override void _ParseGroup(GodotObject @object, string group)
        {
            //GD.Print($"parsing group: {group}");
        }

        public override void _ParseCategory(GodotObject @object, string category)
        {
            if (category == "MMDModel")
            {
                var b1 = AddButton("唯一化此物体所有ShaderMaterial", "这可以让你在复制物体后使用不同的材质，而不影响其他物体的材质。");
                b1.Pressed += () =>
                {
                    ReplaceMaterialAction replaceMaterial = new ReplaceMaterialAction();
                    replaceMaterial.DuplicateMaterials(currentModel, MMDImport.currentPlugin.GetUndoRedo());
                };

                var b3 = AddButton("为当前Pose创建网格", "用于简化特效制作。");
                b3.Pressed += () =>
                {
                    CreatePoseMeshAction action = new CreatePoseMeshAction();
                    action.Do(currentModel, MMDImport.currentPlugin.GetUndoRedo());
                };


                var resourcePicker = new EditorResourcePicker
                {
                    BaseType = "Material",
                    CustomMinimumSize = new Vector2(100, 0)
                };
                var b4 = new Button();
                b4.Text = "设置材质覆盖层";
                b4.TooltipText = "设置此物体的材质覆盖层，用于特效。";
                b4.Pressed += () =>
                {
                    SetMeshOverlayAction setMeshOverlay = new SetMeshOverlayAction();
                    setMeshOverlay.Do(currentModel, (Material)resourcePicker.EditedResource, MMDImport.currentPlugin.GetUndoRedo());
                };
                var hBoxContainer = new HBoxContainer
                {
                    Alignment = BoxContainer.AlignmentMode.Center
                };
                hBoxContainer.AddChild(resourcePicker);
                hBoxContainer.AddChild(b4);
                AddCustomControl(hBoxContainer);
            }
        }

        Button AddButton(string text, string tooltip = "")
        {
            var button = new Button();
            button.Text = text;
            button.TooltipText = tooltip;
            AddCustomControl(button);
            return button;
        }
    }
}
