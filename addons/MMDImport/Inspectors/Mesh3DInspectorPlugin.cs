using Godot;

namespace Mmd.addons.MMDImport.Inspectors
{
    public partial class Mesh3DInspectorPlugin : EditorInspectorPlugin
    {

        public override bool _CanHandle(GodotObject @object)
        {
            if (@object is MeshInstance3D)
                return true;
            return false;
        }

        readonly string materialOverridePath = "surface_material_override/";
        public override bool _ParseProperty(GodotObject @object, Variant.Type type, string path, PropertyHint hintType, string hintString, PropertyUsageFlags usageFlags, bool wide)
        {
            if (path.StartsWith(materialOverridePath))
            {
                string a = path[materialOverridePath.Length..^0];

                var hBoxContainer = new HBoxContainer
                {
                    Alignment = BoxContainer.AlignmentMode.Center
                };
                var resourcePicker = new EditorResourcePicker
                {
                    BaseType = "Material",
                    CustomMinimumSize = new Vector2(100, 0)
                };
                hBoxContainer.AddChild(new Label { Text = $"使用材质预设替换{a}" });
                hBoxContainer.AddChild(resourcePicker);
                resourcePicker.ResourceChanged += (resource) =>
                {
                    if (resource == null)
                        return;

                    var material = (Material)@object.Get(path);
                    var duplicatedMaterial = (Material)resource.Duplicate();
                    if (duplicatedMaterial is ShaderMaterial sm)
                    {
                        if (material is StandardMaterial3D standardMaterial)
                        {
                            sm.SetShaderParameter("texture_albedo", standardMaterial.AlbedoTexture);
                            sm.SetShaderParameter("texture_normal", standardMaterial.NormalTexture);
                            sm.RenderPriority = standardMaterial.RenderPriority;
                        }
                        else if (material is ShaderMaterial shaderMaterial)
                        {
                            sm.SetShaderParameter("texture_albedo", shaderMaterial.GetShaderParameter("texture_albedo"));
                            sm.SetShaderParameter("texture_normal", shaderMaterial.GetShaderParameter("texture_normal"));
                            sm.RenderPriority = shaderMaterial.RenderPriority;
                        }
                    }
                    else if (duplicatedMaterial is StandardMaterial3D sm1)
                    {
                        if (material is StandardMaterial3D standardMaterial)
                        {
                            sm1.AlbedoTexture = standardMaterial.AlbedoTexture;
                            sm1.NormalTexture = standardMaterial.NormalTexture;
                            sm1.RenderPriority = standardMaterial.RenderPriority;
                        }
                        else if (material is ShaderMaterial shaderMaterial)
                        {
                            sm1.AlbedoTexture = shaderMaterial.GetShaderParameter("texture_albedo").As<Texture2D>();
                            sm1.NormalTexture = shaderMaterial.GetShaderParameter("texture_normal").As<Texture2D>();
                            sm1.RenderPriority = shaderMaterial.RenderPriority;
                        }
                    }
                    ReplaceMaterialAction replaceMaterial = new ReplaceMaterialAction();
                    replaceMaterial.ReplaceMaterialWithPreset(((Node3D)@object).GetParent(), material, duplicatedMaterial, MMDImport.currentPlugin.GetUndoRedo());
                };

                AddCustomControl(hBoxContainer);
            }
            return false;
        }
    }
}
