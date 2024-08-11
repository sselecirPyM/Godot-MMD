#if TOOLS
using Godot;
using Godot.Collections;

namespace Mmd.addons.MMDImport
{
    [Tool]
    public partial class VMDImporter : EditorImportPlugin
    {
        public override string _GetImporterName()
        {
            return "mmd.vmd_importer";
        }

        public override string _GetVisibleName()
        {
            return "VMD File";
        }

        public override string[] _GetRecognizedExtensions()
        {
            return new string[] { "vmd" };
        }

        public override string _GetSaveExtension()
        {
            return "res";
        }

        public override string _GetResourceType()
        {
            return "Resource";
        }

        public override int _GetPresetCount()
        {
            return 0;
        }
        public override Godot.Collections.Array<Godot.Collections.Dictionary> _GetImportOptions(string path, int presetIndex)
        {
            return new Godot.Collections.Array<Godot.Collections.Dictionary>
            {
            };
        }

        public override float _GetPriority()
        {
            return 1.0f;
        }

        public override string _GetPresetName(int presetIndex)
        {
            return "Default";
        }

        public override int _GetImportOrder()
        {
            return 0;
        }

        public override Error _Import(string sourceFile, string savePath, Dictionary options, Array<string> platformVariants, Array<string> genFiles)
        {
            string filename = $"{savePath}.{_GetSaveExtension()}";
            var bytes = Godot.FileAccess.GetFileAsBytes(sourceFile);
            var vmd = VMDFormat.Load(bytes);
            if (vmd.CameraKeyFrames.Count > 0)
            {
                var vmdCameraResource = new VMDCameraResource();
                vmdCameraResource.Data = bytes;
                vmdCameraResource.ResourceName = vmd.Name;
                var last = vmd.CameraKeyFrames[^1];
                vmdCameraResource.Duration = (double)last.Frame / 30.0;


                return ResourceSaver.Save(vmdCameraResource, filename);
            }
            else
            {
                var vmdResource = new VMDResource();
                vmdResource.Data = bytes;
                vmdResource.ResourceName = vmd.Name;

                foreach (var f in vmd.BoneKeyFrameSet)
                {
                    var last = f.Value[^1];
                    vmdResource.Duration = Mathf.Max(vmdResource.Duration, last.Frame / 30.0);
                }

                return ResourceSaver.Save(vmdResource, filename);
            }
        }
    }
}
#endif